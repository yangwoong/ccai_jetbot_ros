import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class DepthNavNode(Node):
    """Reactive obstacle avoidance + open-direction steering driven by a
    front-facing Intel RealSense D435i's real depth data, instead of the
    monocular CSI heuristics (edge density / floor color / frame-change) that
    vision_nav_node uses. Those heuristics were built because there was no
    depth sensor; with the D435i physically measuring distance, obstacle
    detection no longer has to be inferred indirectly from texture/color and
    is far less prone to the false positives the CSI-only approach hit (see
    docs/vision_and_alerts.md).

    This node publishes to the exact same /ccai/vision_cmd_vel and
    /ccai/vision_status topics vision_nav_node uses, so patrol_node needs no
    changes - it already just drives whatever the most recent vision_cmd_vel
    is. Only one of the two should actually be driving at a time: when this
    node is enabled (D435i connected, pointed forward), set
    vision_nav_node.drive_enabled: false in robot.yaml so the CSI node
    continues doing YOLO object recognition / follow-person duty (its camera
    is ceiling-mounted now and can't usefully see the floor ahead) without
    also fighting this node for the drive topic.

    It also publishes an annotated color-camera preview to
    /ccai/depth_debug_image (drivable-path bands colored by openness,
    obstacle/clear state, and the robot's current mode/location label) so the
    web UI can show what the D435i is actually seeing and deciding, the same
    way vision_nav_node's debug overlay does for CSI.

    This is a reactive "seek open space, back off from what's close" patrol
    behavior, not full SLAM/occupancy-grid navigation - there is still no
    odometry or map here. See docs/navigation_roadmap.md for what a further
    RTAB-Map/Nav2 phase on top of this would add.
    """

    def __init__(self) -> None:
        super().__init__("depth_nav_node")
        self.declare_parameter("enabled", False)
        # Set this true whenever patrol_node owns EXPLORING driving itself -
        # either its explore_frontier_mode (coverage-seeking) or
        # explore_room_scan_mode (timed rotation + VLM doorway detection, see
        # patrol_node.py's tick_room_scan). Despite the name (kept from when
        # only one alternate mode existed), it just means "don't drive or
        # compete during EXPLORING here." Without this, this node kept
        # computing and publishing its own competing /ccai/vision_cmd_vel +
        # obstacle_now during EXPLORING regardless of which patrol_node mode
        # was active, and patrol_node's obstacle-safety override would then
        # adopt that twist wholesale any time anything was nearby - in
        # practice, almost permanently masking the new algorithm with this
        # node's old reactive one (symptom: "algorithm looks unchanged, robot
        # just spins in place" - confirmed on real hardware 2026-07-25).
        self.declare_parameter("explore_frontier_mode", False)
        self.declare_parameter("depth_image_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("color_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("debug_image_enabled", True)
        self.declare_parameter("depth_scale_to_meters", 0.001)
        self.declare_parameter("min_valid_depth_m", 0.2)
        self.declare_parameter("max_valid_depth_m", 4.0)
        self.declare_parameter("linear_speed", 0.045)
        self.declare_parameter("turn_speed", 0.16)
        self.declare_parameter("max_angular_speed", 0.22)
        self.declare_parameter("obstacle_stop_distance_m", 0.45)
        # The debug overlay's ROI band (0.35-0.75 of frame height, see
        # build_drivable_overlay) still shows perfectly clear floor as red
        # near the bottom of that band - confirmed on real hardware. Simple
        # camera-angle geometry: rows further down the image look at a
        # steeper downward angle, so genuinely empty floor there measures
        # closer than floor near the top of the band, regardless of any real
        # obstacle. A single flat obstacle_stop_distance_m across the whole
        # band can't tell that apart from an actual close object. This
        # factor tapers the overlay-only red/green threshold down linearly
        # across the band (1.0 = full obstacle_stop_distance_m at the top of
        # the band, this factor * obstacle_stop_distance_m at the bottom),
        # compensating for that perspective effect so clear floor reads
        # green all the way down. Purely a display heuristic - does not
        # touch obstacle_now/analyze_depth's actual driving decision.
        self.declare_parameter("overlay_near_stop_factor", 0.5)
        self.declare_parameter("min_valid_frame_seconds", 1.0)
        self.declare_parameter("obstacle_avoidance_hold_seconds", 1.0)
        self.declare_parameter("obstacle_clear_confirm_frames", 5)
        self.declare_parameter("obstacle_avoidance_max_seconds", 6.0)
        self.declare_parameter("obstacle_turn_pulse_seconds", 0.3)
        self.declare_parameter("obstacle_pause_seconds", 0.2)
        self.declare_parameter("steer_smoothing_alpha", 0.4)
        self.declare_parameter("speed_ramp_seconds", 1.5)
        self.declare_parameter("speed_ramp_min_factor", 0.35)
        self.declare_parameter("camera_alert_min_interval_seconds", 10.0)

        self.np = None
        self.cv2 = None
        self.cv_bridge = None
        self.mode = "idle"
        self.target = ""
        self.last_valid_frame_at = 0.0
        self.forward_streak_started_at = 0.0
        self.obstacle_avoidance_direction = 0
        self.obstacle_avoidance_until = 0.0
        self.obstacle_avoidance_started_at = 0.0
        self.obstacle_clear_streak = 0
        self.smoothed_steer = 0.0
        self.event_throttle_at = {}
        self.last_signals = None
        self.last_detail = ""
        self.last_frame_count = 0
        self.last_depth_frame = None

        self.cmd_pub = self.create_publisher(Twist, "/ccai/vision_cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/ccai/vision_status", 10)
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.debug_image_pub = self.create_publisher(CompressedImage, "/ccai/depth_debug_image", 2)

        if not bool(self.get_parameter("enabled").value):
            self.get_logger().info("depth_nav_node disabled (enable via 'enabled' param once D435i is connected)")
            return

        try:
            import cv2
            import numpy as np
            from cv_bridge import CvBridge

            self.np = np
            self.cv2 = cv2
            self.cv_bridge = CvBridge()
        except Exception as exc:
            self.publish_event("depth_nav_node unavailable: {0}".format(exc))
            return

        depth_topic = str(self.get_parameter("depth_image_topic").value)
        color_topic = str(self.get_parameter("color_image_topic").value)
        self.create_subscription(Image, depth_topic, self.on_depth_image, 2)
        self.create_subscription(Image, color_topic, self.on_color_image, 2)
        self.create_subscription(String, "/ccai/status", self.on_robot_status, 10)
        self.create_timer(0.5, self.watchdog)
        self.publish_event("depth_nav_node ready, depth_topic={0}, color_topic={1}".format(depth_topic, color_topic))

    def on_robot_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        self.mode = str(payload.get("state", "idle"))
        self.target = str(payload.get("target", ""))

    def on_depth_image(self, msg: Image) -> None:
        try:
            depth = self.decode_depth(msg)
        except Exception as exc:
            self.publish_event_throttled("depth frame decode failed: {0}".format(exc), key="depth_decode")
            return
        if depth is None:
            return

        self.last_valid_frame_at = time.monotonic()
        self.last_frame_count += 1
        signals = self.analyze_depth(depth)
        self.last_signals = signals
        self.last_depth_frame = depth

        frontier_mode = bool(self.get_parameter("explore_frontier_mode").value)
        # pose_goal is always excluded from driving here: patrol_node's own
        # point-to-point controller owns POSE_GOAL, this node is sensor-only
        # for that state (same reasoning as the exploring+frontier_mode case
        # below - see this node's declare_parameter comment for
        # explore_frontier_mode).
        drives_forward = self.mode == "patrolling" or (
            self.mode == "exploring" and not frontier_mode
        ) or (self.mode == "manual_drive" and self.target == "move_forward")
        if drives_forward:
            twist, detail = self.compute_patrol_command(signals)
            self.last_detail = detail
            self.cmd_pub.publish(twist)
            self.publish_status("patrol", detail=detail)
        else:
            self.last_detail = self.describe_signals(signals, suffix=" (not driving)")
            # Still publish status (with a fresh obstacle_now) even though this
            # node isn't driving right now - patrol_node's POSE_GOAL/frontier
            # EXPLORING controllers consult obstacle_now as their own safety
            # check and need it to stay live, not frozen at whatever it was
            # when this node was last actually driving.
            self.publish_status(self.mode, detail=self.last_detail)

    def on_color_image(self, msg: Image) -> None:
        if not bool(self.get_parameter("debug_image_enabled").value):
            return
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.publish_event_throttled("color frame decode failed: {0}".format(exc), key="color_decode")
            return
        self.publish_debug_frame(frame)

    def decode_depth(self, msg: Image):
        """Convert the raw depth Image to a meters-per-pixel numpy array. D435i
        publishes 16UC1 (millimeters as raw integer counts by default), scaled
        by depth_scale_to_meters - adjust that parameter if a different depth
        unit is configured on the camera.
        """
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        scale = float(self.get_parameter("depth_scale_to_meters").value)
        return frame.astype(self.np.float32) * scale

    def region_distance(self, region) -> float:
        """Median of valid (non-zero = no-return, within sensor range) depth
        readings in a region, in meters. Returns max_valid_depth_m (i.e. "wide
        open") if there's no valid reading at all, rather than treating a
        no-return patch (common on featureless walls/floors just past sensor
        range) as if it were an obstacle at distance zero.
        """
        min_valid = float(self.get_parameter("min_valid_depth_m").value)
        max_valid = float(self.get_parameter("max_valid_depth_m").value)
        valid = region[(region >= min_valid) & (region <= max_valid)]
        if valid.size > 0:
            return float(self.np.median(valid))
        # Nothing landed in the trustworthy range - this covers two very
        # different situations that both used to fall through to "wide open"
        # (max_valid), which let the robot keep driving straight into
        # whatever it was approaching (confirmed on real hardware: obstacle
        # correctly detected while still a bit away, then NOT avoided,
        # collision - because right at the closest point, depth readings go
        # below the D435i's ~0.2m minimum reliable range and read as
        # near-zero/no-return, exactly like "nothing here"). Distinguish by
        # checking for a cluster of positive-but-below-min_valid readings,
        # which only happens when something is actually there and too close
        # to measure - a genuine no-return (far wall past max_valid,
        # featureless surface) has no such cluster.
        # "바닥 장애물 인식을 너무 못해" (2026-07-27) - lowered from 0.1: a
        # thin/low obstacle (cable, book, shoe) only fills a small fraction
        # of the region with too-close pixels, and the old 10% threshold let
        # those get missed and fall through to "wide open".
        too_close = region[(region > 0.0) & (region < min_valid)]
        if too_close.size > region.size * 0.04:
            return 0.0
        return max_valid

    def analyze_depth(self, depth) -> dict:
        height, width = depth.shape[:2]
        # "바닥 장애물 인식을 너무 못해. 주행뷰에서 장애물 인식률을 올려"
        # (2026-07-27): band widened from 0.75 to 0.85 of frame height to
        # catch low floor obstacles closer to the robot that the previous,
        # more conservative cutoff missed entirely. This is a patrol robot
        # that doesn't need to hug walls or squeeze through tight gaps (see
        # obstacle_stop_distance_m's comment) - occasionally stopping a
        # little early for perspective-close-looking floor is an acceptable
        # trade for catching more real obstacles.
        band = depth[int(height * 0.35): int(height * 0.85), :]
        third = width // 3
        left_distance = self.region_distance(band[:, :third])
        center_distance = self.region_distance(band[:, third: 2 * third])
        right_distance = self.region_distance(band[:, 2 * third:])
        stop_distance = float(self.get_parameter("obstacle_stop_distance_m").value)
        # Any of the three columns (not just center) triggers a stop - a
        # narrow robot can still clip something off-center, and again, this
        # patrol robot doesn't need to shave margin for extra maneuverability.
        obstacle_now = min(left_distance, center_distance, right_distance) < stop_distance
        return {
            "left_distance": left_distance,
            "center_distance": center_distance,
            "right_distance": right_distance,
            "obstacle_now": obstacle_now,
        }

    def describe_signals(self, signals: dict, suffix: str = "") -> str:
        return "depth path left={0:.2f}m center={1:.2f}m right={2:.2f}m{3}".format(
            signals["left_distance"], signals["center_distance"], signals["right_distance"], suffix
        )

    def compute_patrol_command(self, signals: dict):
        left_distance = signals["left_distance"]
        center_distance = signals["center_distance"]
        right_distance = signals["right_distance"]
        obstacle_now = signals["obstacle_now"]

        twist = Twist()
        turn_speed = float(self.get_parameter("turn_speed").value)
        now = time.monotonic()

        hold_seconds = float(self.get_parameter("obstacle_avoidance_hold_seconds").value)
        confirm_frames = max(int(self.get_parameter("obstacle_clear_confirm_frames").value), 1)

        if obstacle_now:
            self.obstacle_clear_streak = 0
            was_idle = self.obstacle_avoidance_direction == 0
            if was_idle or now >= self.obstacle_avoidance_until:
                # Real metric distance, so unlike the CSI edge-density proxy
                # this comparison is trustworthy every frame - still commit to
                # one direction per episode for a clean, non-flappy turn.
                self.obstacle_avoidance_direction = -1 if left_distance < right_distance else 1
            if was_idle:
                self.obstacle_avoidance_started_at = now
            self.obstacle_avoidance_until = now + hold_seconds

            max_seconds = float(self.get_parameter("obstacle_avoidance_max_seconds").value)
            if self.obstacle_avoidance_started_at > 0.0 and now - self.obstacle_avoidance_started_at > max_seconds:
                self.obstacle_avoidance_direction = 0
                self.obstacle_avoidance_started_at = 0.0
                self.obstacle_clear_streak = 0
                self.forward_streak_started_at = 0.0
                detail = "depth obstacle avoidance timed out after {0:.1f}s, stopping".format(max_seconds)
                self.publish_event_throttled(detail, key="avoidance_timeout")
                return twist, detail

            pulse = max(float(self.get_parameter("obstacle_turn_pulse_seconds").value), 0.05)
            pause = max(float(self.get_parameter("obstacle_pause_seconds").value), 0.0)
            cycle = pulse + pause
            in_turn_phase = (now % cycle) < pulse
            twist.angular.z = turn_speed * self.obstacle_avoidance_direction if in_turn_phase else 0.0
            detail = "depth obstacle center={0:.2f}m left={1:.2f}m right={2:.2f}m dir={3:+d}".format(
                center_distance, left_distance, right_distance, self.obstacle_avoidance_direction
            )
            self.forward_streak_started_at = 0.0
            return twist, detail

        self.obstacle_clear_streak += 1
        if self.obstacle_avoidance_direction != 0 and (now < self.obstacle_avoidance_until or self.obstacle_clear_streak < confirm_frames):
            pulse = max(float(self.get_parameter("obstacle_turn_pulse_seconds").value), 0.05)
            pause = max(float(self.get_parameter("obstacle_pause_seconds").value), 0.0)
            cycle = pulse + pause
            in_turn_phase = (now % cycle) < pulse
            twist.angular.z = turn_speed * self.obstacle_avoidance_direction if in_turn_phase else 0.0
            detail = "depth clearing obstacle: confirming clear ({0}/{1})".format(self.obstacle_clear_streak, confirm_frames)
            return twist, detail
        self.obstacle_avoidance_direction = 0
        self.obstacle_avoidance_started_at = 0.0

        if self.forward_streak_started_at <= 0.0:
            self.forward_streak_started_at = now
        ramp_seconds = float(self.get_parameter("speed_ramp_seconds").value)
        min_factor = float(self.get_parameter("speed_ramp_min_factor").value)
        ramp_factor = clamp((now - self.forward_streak_started_at) / max(ramp_seconds, 0.01), min_factor, 1.0)

        # Steer toward whichever side has more open space - genuine
        # "seek the clearer path" exploration behavior rather than a fixed
        # route, since there is still no map/localization to plan against.
        max_valid = float(self.get_parameter("max_valid_depth_m").value)
        steer_raw = clamp((right_distance - left_distance) / max_valid, -1.0, 1.0)
        smoothing_alpha = float(self.get_parameter("steer_smoothing_alpha").value)
        self.smoothed_steer = smoothing_alpha * steer_raw + (1.0 - smoothing_alpha) * self.smoothed_steer
        steer = self.smoothed_steer

        twist.linear.x = float(self.get_parameter("linear_speed").value) * ramp_factor
        twist.angular.z = clamp(steer * float(self.get_parameter("max_angular_speed").value), -0.22, 0.22)
        detail = "depth path left={0:.2f}m center={1:.2f}m right={2:.2f}m steer={3:.2f} ramp={4:.2f}".format(
            left_distance, center_distance, right_distance, steer, ramp_factor
        )
        return twist, detail

    def build_drivable_overlay(self, depth, shape) -> object:
        """Drivable-floor overlay: raw per-pixel depth classification produced
        scattered, noisy patches (confirmed on real hardware - a speckled mix
        of tiny red/yellow/green fragments, not a clean readable shape). This
        cleans that up into a single smooth green blob for "where it's safe
        to drive" - open-floor pixels are morphologically closed/opened
        (fills small gaps, drops speckle noise) and only the single largest
        connected region is kept and filled, discarding smaller disconnected
        fragments. Close-range pixels are drawn as solid red on top (no
        separate "caution" yellow tier - it added visual noise without being
        used by any driving logic, this overlay is purely for the human
        watching the web UI). Returns an RGB image of `shape` with 0 alpha
        (black) outside the analysis region, to be alpha-blended by the caller.
        """
        height, width = shape[:2]
        if depth.shape[:2] != (height, width):
            depth = self.cv2.resize(depth, (width, height), interpolation=self.cv2.INTER_NEAREST)

        min_valid = float(self.get_parameter("min_valid_depth_m").value)
        max_valid = float(self.get_parameter("max_valid_depth_m").value)
        stop_distance = float(self.get_parameter("obstacle_stop_distance_m").value)

        # Same band as analyze_depth() uses for the actual driving decision
        # (0.35-0.85 of the frame height, widened from 0.75 on 2026-07-27 to
        # improve floor-obstacle recall - see analyze_depth's comment) - NOT
        # just "top cropped, bottom included". Including the very bottom of
        # the frame (floor right under the robot) still paints it red even
        # when clearly drivable, because that floor is always the closest
        # point in the whole depth image by simple camera geometry (steepest
        # viewing angle = shortest range), regardless of whether anything is
        # actually blocking it - it's not an obstacle, it's perspective.
        # Excluding just that very bottom sliver (and matching analyze_depth's
        # own band otherwise) keeps this overlay honest, and keeps it
        # visually consistent with the band that's actually driving the robot.
        roi_top = int(height * 0.35)
        roi_bottom = int(height * 0.85)
        valid = (depth >= min_valid) & (depth <= max_valid)
        valid[:roi_top, :] = False
        valid[roi_bottom:, :] = False

        # Taper the red/green threshold across the band instead of one flat
        # value - see overlay_near_stop_factor's declare_parameter comment
        # (perspective alone makes clear floor read closer near the bottom
        # of the band).
        near_factor = float(self.get_parameter("overlay_near_stop_factor").value)
        row_idx = self.np.arange(height, dtype=self.np.float32).reshape(-1, 1)
        band_span = max(roi_bottom - roi_top, 1)
        row_frac = self.np.clip((row_idx - roi_top) / band_span, 0.0, 1.0)
        stop_map = stop_distance * (1.0 - row_frac * (1.0 - near_factor))

        red_mask = valid & (depth <= stop_map)
        open_floor = valid & (depth > stop_map)

        green_mask = self.np.zeros((height, width), dtype=bool)
        open_u8 = (open_floor.astype(self.np.uint8)) * 255
        kernel = self.np.ones((15, 15), self.np.uint8)
        cleaned = self.cv2.morphologyEx(open_u8, self.cv2.MORPH_OPEN, kernel)
        cleaned = self.cv2.morphologyEx(cleaned, self.cv2.MORPH_CLOSE, kernel)
        contours, _ = self.cv2.findContours(cleaned, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=self.cv2.contourArea)
            # Ignore tiny noise contours (< 1% of the frame) - only draw a
            # blob big enough to actually mean "there's a way through here".
            if self.cv2.contourArea(largest) > height * width * 0.01:
                mask_u8 = self.np.zeros((height, width), dtype=self.np.uint8)
                self.cv2.drawContours(mask_u8, [largest], -1, 255, thickness=self.cv2.FILLED)
                green_mask = mask_u8 > 0

        overlay = self.np.zeros((height, width, 3), dtype=self.np.uint8)
        overlay[green_mask] = (0, 200, 0)
        overlay[red_mask] = (0, 0, 255)
        mask = green_mask | red_mask
        return overlay, mask

    def publish_debug_frame(self, frame) -> None:
        """Draw the per-pixel drivable-floor overlay, current obstacle/
        steering state, and the robot's current mode/location label onto the
        D435i color frame, so the web UI's main preview shows not just raw
        video but what the navigation is actually seeing and deciding -
        mirrors vision_nav_node's CSI debug overlay.
        """
        try:
            signals = self.last_signals
            debug = frame.copy()
            height, width = debug.shape[:2]

            if self.last_depth_frame is not None:
                overlay, mask = self.build_drivable_overlay(self.last_depth_frame, debug.shape)
                blended = self.cv2.addWeighted(overlay, 0.4, debug, 0.6, 0)
                debug[mask] = blended[mask]

            if signals is not None:
                status_text = "OBSTACLE" if signals["obstacle_now"] else "CLEAR"
                status_color = (0, 0, 255) if signals["obstacle_now"] else (0, 200, 0)
                metrics = "left={0:.2f}m center={1:.2f}m right={2:.2f}m".format(
                    signals["left_distance"], signals["center_distance"], signals["right_distance"]
                )
                self.cv2.putText(debug, metrics, (8, height - 50), self.cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            else:
                status_text = "NO SIGNAL"
                status_color = (0, 165, 255)

            self.cv2.putText(debug, status_text, (8, 24), self.cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

            # Navigation/location label - shows what the robot currently
            # believes it's doing (patrolling, heading to a taught location by
            # name during REPLAYING, manual drive, etc.) directly on the feed.
            label = "mode={0}".format(self.mode)
            if self.target:
                label += " target={0}".format(self.target)
            self.cv2.putText(debug, label, (8, height - 34), self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            if self.last_detail:
                self.cv2.putText(
                    debug, self.last_detail[:90], (8, height - 12), self.cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1
                )

            ok, encoded = self.cv2.imencode(".jpg", debug, [int(self.cv2.IMWRITE_JPEG_QUALITY), 60])
            if ok:
                msg = CompressedImage()
                msg.format = "jpeg"
                msg.data = encoded.tobytes()
                self.debug_image_pub.publish(msg)
        except Exception as exc:
            self.get_logger().debug("depth debug frame draw failed: {0}".format(exc))

    def watchdog(self) -> None:
        drives_forward = self.mode in ("patrolling", "exploring", "pose_goal") or (self.mode == "manual_drive" and self.target == "move_forward")
        if not drives_forward:
            return
        timeout = float(self.get_parameter("min_valid_frame_seconds").value)
        if self.last_valid_frame_at > 0.0 and time.monotonic() - self.last_valid_frame_at > timeout:
            self.cmd_pub.publish(Twist())
            self.publish_status("depth_camera_timeout", stop=True)
            self.publish_event_throttled("D435i depth frames stopped arriving, stopping motion", key="depth_camera")

    def publish_status(self, state: str, detail: str = "", stop: bool = False) -> None:
        # obstacle_now lets other nodes (patrol_node's point-to-point
        # controller) check "is it safe to drive forward right now" directly
        # instead of string-parsing `detail`.
        obstacle_now = bool(self.last_signals["obstacle_now"]) if self.last_signals is not None else False
        payload = {
            "state": state, "detail": detail, "stop": stop, "mode": self.mode, "target": self.target,
            "obstacle_now": obstacle_now,
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_event_throttled(self, text: str, key: str = "default") -> None:
        min_interval = float(self.get_parameter("camera_alert_min_interval_seconds").value)
        now = time.monotonic()
        last_at = self.event_throttle_at.get(key, 0.0)
        if now - last_at < min_interval:
            return
        self.event_throttle_at[key] = now
        self.publish_event(text)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
