import json
import time
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from ccai_jetbot_patrol.locations import LocationStore
from ccai_jetbot_patrol.mission import parse_mission_command


class PatrolState(str, Enum):
    IDLE = "idle"
    PATROLLING = "patrolling"
    FOLLOWING_PERSON = "following_person"
    INSPECTING = "inspecting"
    RETURNING_HOME = "returning_home"
    STOPPED = "stopped"
    MANUAL = "manual"
    MANUAL_DRIVE = "manual_drive"
    REPLAYING = "replaying"
    EXPLORING = "exploring"
    POSE_GOAL = "pose_goal"


class PatrolNode(Node):
    def __init__(self) -> None:
        super().__init__("patrol_node")
        self.declare_parameter("linear_speed", 0.12)
        self.declare_parameter("angular_speed", 0.35)
        self.declare_parameter("heartbeat_seconds", 2.0)
        self.declare_parameter("safe_stop_on_idle", True)
        self.declare_parameter("patrol_forward_seconds", 4.0)
        self.declare_parameter("patrol_turn_seconds", 1.2)
        self.declare_parameter("use_vision_cmd_vel", True)
        self.declare_parameter("vision_command_timeout_seconds", 0.8)
        self.declare_parameter("manual_move_seconds", 1.5)
        self.declare_parameter("manual_turn_seconds", 0.8)
        self.declare_parameter("speed_step", 0.2)
        self.declare_parameter("min_speed_scale", 0.3)
        self.declare_parameter("max_speed_scale", 2.0)
        self.declare_parameter("speed_ramp_seconds", 1.5)
        self.declare_parameter("speed_ramp_min_factor", 0.35)
        self.declare_parameter("manual_drive_slow_factor", 0.5)
        self.declare_parameter("locations_file", "data/locations.json")
        # How often (seconds of exploring, not counting time spent awaiting a
        # reply) the robot stops to ask the admin to name the current spot,
        # and how long it waits for that reply before giving up and moving on.
        self.declare_parameter("explore_label_interval_seconds", 45.0)
        self.declare_parameter("explore_label_timeout_seconds", 120.0)
        # When true, EXPLORING uses visual_odom_node's (x, y, yaw) estimate
        # to actively seek uncovered ground (coverage-seeking - see
        # pick_explore_subgoal) and steers there itself via
        # compute_steering_twist, instead of depth_nav_node's plain reactive
        # "steer toward whichever side looks more open" heuristic (which has
        # no notion of "already been here"). Requires visual_odom_node.enabled:
        # true too, or there's no pose to seek with. Default false: the
        # existing reactive patrol is completely unaffected unless both are
        # explicitly turned on together. Depth-based obstacle safety (from
        # depth_nav_node) still overrides in both modes.
        self.declare_parameter("explore_frontier_mode", False)
        self.declare_parameter("odom_timeout_seconds", 2.0)
        self.declare_parameter("pose_goal_tolerance_m", 0.15)
        self.declare_parameter("pose_goal_timeout_seconds", 30.0)
        self.declare_parameter("heading_align_threshold_rad", 0.3)
        self.declare_parameter("explore_step_distance_m", 0.8)
        self.declare_parameter("explore_visited_cell_size_m", 0.5)
        self.declare_parameter("explore_candidate_count", 8)

        self.state = PatrolState.IDLE
        self.current_target = ""
        self.last_vlm_summary = ""
        self.last_vision_status = ""
        self.last_vision_cmd = Twist()
        self.last_vision_cmd_at = 0.0
        self.state_changed_at = time.monotonic()
        self.manual_kind = ""
        self.manual_drive_slow = False
        self.speed_scale = 1.0
        self.pending_analysis = False
        self.pending_analysis_location = ""
        self.recording = False
        self.record_buffer = []
        self.location_store = LocationStore(str(self.get_parameter("locations_file").value))
        self.replay_steps = []
        self.replay_index = 0
        self.replay_step_started_at = 0.0
        self.replay_question = ""
        self.replay_location = ""
        self.awaiting_label = False
        self.explore_last_label_request_at = 0.0
        # Own-odometry state (from visual_odom_node's /ccai/odom_pose, a
        # lightweight self-built RGB-D visual odometry - see that node's
        # docstring for why this replaces rtabmap/Nav2 here). Everything
        # below is None/unavailable until that node is enabled and running;
        # has_recent_odom() is the single check everything else goes through.
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_received_at = 0.0
        self.pose_goal_target = None
        self.pose_goal_label = ""
        self.pose_goal_question = ""
        self.pose_goal_started_at = 0.0
        self.visited_cells = {}
        self.explore_sub_goal = None
        self.explore_sub_goal_started_at = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/ccai/status", 10)
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)
        self.vlm_trigger_pub = self.create_publisher(String, "/ccai/vlm_trigger", 10)
        self.location_feature_request_pub = self.create_publisher(String, "/ccai/location_feature_request", 10)
        self.create_subscription(String, "/ccai/mission_command", self.on_mission_command, 10)
        self.create_subscription(String, "/ccai/vlm_observation", self.on_vlm_observation, 10)
        self.create_subscription(String, "/ccai/vision_status", self.on_vision_status, 10)
        self.create_subscription(Twist, "/ccai/vision_cmd_vel", self.on_vision_cmd_vel, 10)
        self.create_subscription(String, "/ccai/location_feature_result", self.on_location_feature_result, 10)
        self.create_subscription(String, "/ccai/odom_pose", self.on_odom_pose, 10)
        # Raw admin chat text, subscribed directly (not via the parsed
        # /ccai/mission_command) so a free-form location name typed in reply
        # to a label request ("창고", "휴게실 앞" etc.) can be captured as-is
        # instead of being forced through command parsing, which has no
        # concept of "the next message is a label answer". See on_admin_text.
        self.create_subscription(String, "/ccai/admin_text", self.on_admin_text, 10)
        # Vestigial no-op broadcast from the earlier Nav2-based design - kept
        # harmless (no subscriber now) since nothing currently depends on it.
        self.explore_pause_pub = self.create_publisher(Bool, "/ccai/explore_pause", 10)

        heartbeat = float(self.get_parameter("heartbeat_seconds").value)
        self.create_timer(heartbeat, self.publish_status)
        self.create_timer(0.2, self.drive_loop)
        self.get_logger().info("patrol_node ready")

    def on_mission_command(self, msg: String) -> None:
        command = parse_mission_command(msg.data)
        self.get_logger().info(f"mission command: {command.type}")

        if command.type == "patrol_start":
            self.set_state(PatrolState.PATROLLING)
            self.current_target = ""
            self.publish_event("patrol started")
        elif command.type == "patrol_stop":
            self.set_state(PatrolState.STOPPED)
            self.stop_motion()
            self.publish_event("patrol stopped")
        elif command.type == "go_home":
            self.set_state(PatrolState.RETURNING_HOME)
            self.current_target = "home"
            self.publish_event("returning home")
        elif command.type == "inspect":
            self.start_inspect(command.target, command.text or command.raw)
        elif command.type == "follow_person":
            self.set_state(PatrolState.FOLLOWING_PERSON)
            self.current_target = command.target or "person"
            self.publish_event("following person: {0}".format(self.current_target))
        elif command.type == "status":
            self.publish_status()
            self.publish_event(self.status_text())
        elif command.type in {"move_forward", "move_backward", "turn_left", "turn_right"}:
            self.start_manual_move(command.type, command.target)
        elif command.type == "set_speed":
            self.adjust_speed(command.target)
        elif command.type == "analyze":
            self.request_analysis()
        elif command.type == "remember_start":
            self.recording = True
            self.record_buffer = []
            self.publish_event("remembering location: recording moves until saved (e.g. '정문으로 저장해')")
        elif command.type == "remember_save":
            self.save_recorded_location(command.target)
        elif command.type == "explore_start":
            self.start_explore()
        elif command.type == "explore_stop":
            self.set_state(PatrolState.STOPPED)
            self.stop_motion()
            self.awaiting_label = False
            self.publish_event("autonomous exploration stopped")
        elif command.type == "say":
            self.publish_event(command.text or command.raw)
        else:
            self.publish_event(f"unknown command: {command.raw}")

    def start_inspect(self, target: str, question: str) -> None:
        if target and self.location_store.has(target):
            self.start_replay(target, question)
            return
        self.set_state(PatrolState.INSPECTING)
        self.current_target = target
        if target:
            self.publish_event(
                f"location '{target}' not known yet; inspecting from current position "
                f"(teach it first: '기억 시작' then move there then '{target}으로 저장해')"
            )
        else:
            self.publish_event("inspecting current position")
        self.request_analysis(question, location=target)

    def start_replay(self, label: str, question: str) -> None:
        pose = self.location_store.get_pose(label)
        if pose is not None:
            # A real coordinate exists for this label (saved during
            # autonomous exploration - see on_admin_text) and our own
            # odometry is running: drive there with the point-to-point
            # controller instead of the old blind timed-move-sequence
            # replay, which has no way to correct for drift. Falls through
            # to that replay/inspect path below if odometry isn't available
            # right now (start_pose_goal returns False in that case).
            if self.start_pose_goal(label, question, pose):
                return
        steps = self.location_store.get(label)
        if not steps:
            # This label was quick-saved (visual features only, no '기억 시작'
            # recording - see save_recorded_location) so there's no route to
            # drive there. Inspect from here instead, but say so explicitly
            # rather than silently dropping the location name.
            self.publish_event(
                f"location '{label}' has no travel path (visual-only save) - inspecting from current position instead"
            )
            self.start_inspect("", question)
            return
        self.replay_steps = steps
        self.replay_index = 0
        self.replay_step_started_at = time.monotonic()
        self.replay_question = question
        self.replay_location = label
        self.current_target = label
        self.set_state(PatrolState.REPLAYING)
        self.publish_event(f"heading to {label} ({len(steps)} steps)")

    def start_pose_goal(self, label: str, question: str, pose: dict) -> bool:
        """Drive to a stored (x, y) coordinate using our own point-to-point
        controller (compute_steering_twist), fed by visual_odom_node's
        lightweight odometry - no Nav2/rtabmap needed. Returns False (caller
        should fall back to the timed-replay/inspect path) if there's no
        recent odometry to navigate by.
        """
        if not self.has_recent_odom():
            self.publish_event(f"오도메트리 없음 - '{label}' 좌표 이동 불가, 기존 방식으로 대체")
            return False
        self.pose_goal_target = (float(pose["x"]), float(pose["y"]))
        self.pose_goal_label = label
        self.pose_goal_question = question
        self.pose_goal_started_at = time.monotonic()
        self.current_target = label
        self.set_state(PatrolState.POSE_GOAL)
        self.publish_event(f"'{label}' 좌표로 이동 시작 (x={pose['x']:.2f}, y={pose['y']:.2f})")
        return True

    def finish_pose_goal(self, success: bool) -> None:
        label = self.pose_goal_label
        question = self.pose_goal_question
        self.stop_motion()
        if success:
            self.publish_event(f"'{label}' 도착 (좌표 이동 완료)")
            stored_features = self.location_store.get_features(label)
            if stored_features:
                self.location_feature_request_pub.publish(String(data=json.dumps(
                    {"action": "match", "label": label, "descriptors": stored_features}, ensure_ascii=False,
                )))
            self.request_analysis(question, location=label)
        self.pose_goal_target = None
        self.set_state(PatrolState.STOPPED)

    def compute_steering_twist(self, target_x: float, target_y: float, linear_speed: float, angular_speed: float):
        """Simple proportional point-to-point controller: rotate to face the
        target if the heading error is large, otherwise drive forward
        (slowing near the goal) while making small heading corrections.
        Returns (Twist, arrived_bool). Assumes has_recent_odom() already True.
        """
        import math

        dx = target_x - self.odom_x
        dy = target_y - self.odom_y
        distance = math.hypot(dx, dy)
        tolerance = float(self.get_parameter("pose_goal_tolerance_m").value)
        twist = Twist()
        if distance < tolerance:
            return twist, True
        target_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(target_heading - self.odom_yaw), math.cos(target_heading - self.odom_yaw)
        )
        align_threshold = float(self.get_parameter("heading_align_threshold_rad").value)
        if abs(heading_error) > align_threshold:
            twist.angular.z = clamp(heading_error * 1.5, -angular_speed, angular_speed)
        else:
            twist.linear.x = min(linear_speed, linear_speed * clamp(distance / max(tolerance * 2.0, 0.01), 0.3, 1.0))
            twist.angular.z = clamp(heading_error * 0.8, -angular_speed * 0.5, angular_speed * 0.5)
        return twist, False

    def compute_pose_goal_twist(self, linear_speed: float, angular_speed: float):
        """Returns a Twist to publish, or None if the goal has already been
        finalized (arrived, timed out, or odometry lost) inside this call -
        the caller should just return without publishing anything more."""
        if not self.has_recent_odom():
            self.publish_event(f"오도메트리 신호 끊김 - '{self.pose_goal_label}' 이동 중단")
            self.finish_pose_goal(False)
            return None
        timeout = float(self.get_parameter("pose_goal_timeout_seconds").value)
        if time.monotonic() - self.pose_goal_started_at > timeout:
            self.publish_event(f"'{self.pose_goal_label}' 이동 시간 초과 - 정지")
            self.finish_pose_goal(False)
            return None
        twist, arrived = self.compute_steering_twist(
            self.pose_goal_target[0], self.pose_goal_target[1], linear_speed, angular_speed
        )
        if arrived:
            self.finish_pose_goal(True)
            return None
        return twist

    def save_recorded_location(self, label: str) -> None:
        if not label:
            self.publish_event("save failed: no location name given")
            return
        if self.recording and self.record_buffer:
            self.location_store.set(label, self.record_buffer)
            self.publish_event(f"location saved: {label} ({len(self.record_buffer)} steps, with return path)")
        else:
            # No '기억 시작' -> move -> save sequence was done, so there's no
            # replayable path to this spot - but the admin is standing here right
            # now saying "remember this place", and we can still remember what it
            # looks like from here via visual features alone (see the capture
            # request below). This is what makes a plain "여기는 작은방이야, 저장해"
            # (with no prior recording) actually save something instead of
            # silently failing with "no recorded moves to save".
            if not self.location_store.has(label):
                self.location_store.set(label, [])
            self.publish_event(
                f"location saved: {label} (현재 위치의 시각 특징만 기억됨, 이동 경로 없음 - "
                "경로도 저장하려면 '기억 시작' 후 이동하고 다시 저장하세요)"
            )
        self.recording = False
        self.record_buffer = []
        self.request_location_feature_capture(label)

    def request_location_feature_capture(self, label: str) -> None:
        # Capture visual features of the current view, so future arrivals at
        # this label can be visually confirmed instead of trusting the timed
        # move-sequence alone (which drifts with no odometry to correct it) -
        # or, for exploration-discovered spots, so it's recognizable at all
        # since there's no move-sequence for those in the first place.
        self.location_feature_request_pub.publish(
            String(data=json.dumps({"action": "capture", "label": label}, ensure_ascii=False))
        )

    def start_explore(self) -> None:
        # Autonomous exploration. By default (explore_frontier_mode: false)
        # this reuses the exact same reactive driving as PATROLLING
        # (vision_nav_node/depth_nav_node both drive "exploring" the same way
        # they drive "patrolling"). When explore_frontier_mode is true and
        # visual_odom_node is running, drive_loop instead uses
        # compute_explore_twist() - a coverage-seeking controller that tracks
        # which coarse grid cells have already been visited (self.visited_cells)
        # and steers toward less-visited ground, which is what actually makes
        # this "look for the way forward" instead of just avoiding whatever's
        # nearby. Either way, patrol_node periodically pauses to build up a
        # set of named locations - see tick_explore_labeling/on_admin_text.
        # Still no real map/loop-closure, so this is a growing list of "known
        # spots" (each with visual features, and a coordinate if odometry is
        # running), not a geometric map - see docs/navigation_roadmap.md.
        self.set_state(PatrolState.EXPLORING)
        self.current_target = ""
        self.awaiting_label = False
        self.explore_last_label_request_at = time.monotonic()
        self.visited_cells = {}
        self.explore_sub_goal = None
        self.publish_event(
            "autonomous exploration started - driving and avoiding obstacles on its own; "
            "periodically stops to ask for a name for the current spot"
        )

    def tick_explore_labeling(self) -> None:
        if self.state != PatrolState.EXPLORING:
            return
        now = time.monotonic()
        if self.awaiting_label:
            timeout = float(self.get_parameter("explore_label_timeout_seconds").value)
            if now - self.explore_last_label_request_at > timeout:
                self.publish_event("라벨 응답이 없어 이 지점은 건너뛰고 탐색을 계속합니다")
                self.awaiting_label = False
                self.explore_last_label_request_at = now
            return
        interval = float(self.get_parameter("explore_label_interval_seconds").value)
        if now - self.explore_last_label_request_at > interval:
            self.awaiting_label = True
            self.explore_last_label_request_at = now
            self.stop_motion()
            self.explore_pause_pub.publish(Bool(data=True))
            self.publish_event(
                "탐색 중 새 지점 발견 - 이 위치의 이름을 답장으로 알려주세요 (건너뛰려면 '스킵')"
            )

    def on_odom_pose(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self.odom_x = float(payload["x"])
            self.odom_y = float(payload["y"])
            self.odom_yaw = float(payload["yaw"])
            self.odom_received_at = time.monotonic()
        except Exception:
            pass

    def has_recent_odom(self) -> bool:
        timeout = float(self.get_parameter("odom_timeout_seconds").value)
        return self.odom_received_at > 0.0 and time.monotonic() - self.odom_received_at <= timeout

    def is_obstacle_now(self) -> bool:
        try:
            payload = json.loads(self.last_vision_status)
            return bool(payload.get("obstacle_now", False))
        except Exception:
            return False

    def on_admin_text(self, msg: String) -> None:
        if self.state != PatrolState.EXPLORING or not self.awaiting_label:
            return
        text = msg.data.strip()
        if not text:
            return
        if text.lower() in {"스킵", "skip", "pass", "넘어가", "건너뛰기", "건너뛰어"}:
            self.publish_event("라벨링 건너뜀 - 탐색 계속")
        else:
            label = text
            if not self.location_store.has(label):
                self.location_store.set(label, [])
            self.publish_event(f"탐색 중 위치 라벨 저장: {label}")
            self.request_location_feature_capture(label)
            if self.has_recent_odom():
                self.location_store.set_pose(label, self.odom_x, self.odom_y, self.odom_yaw)
                self.publish_event(
                    f"'{label}' 좌표 저장됨 (x={self.odom_x:.2f}, y={self.odom_y:.2f}) - "
                    "이 좌표로 나중에 실제 이동 가능"
                )
        self.awaiting_label = False
        self.explore_last_label_request_at = time.monotonic()
        self.explore_pause_pub.publish(Bool(data=False))

    def pick_explore_subgoal(self):
        """Sample candidate points in a ring around the robot and pick
        whichever lands in the least-visited coarse grid cell (with a little
        random tie-breaking so it doesn't lock onto one direction forever) -
        a simple coverage-seeking heuristic that actually answers "which way
        haven't I been" instead of depth_nav_node's plain "which way looks
        more open right now" (the latter has no memory, so it can circle the
        same open room indefinitely - this is the actual bug the user
        reported: obstacles avoided fine, but no progress finding new ground).
        """
        import math
        import random

        cell_size = float(self.get_parameter("explore_visited_cell_size_m").value)
        step = float(self.get_parameter("explore_step_distance_m").value)
        count = max(int(self.get_parameter("explore_candidate_count").value), 1)
        best = None
        best_score = None
        for i in range(count):
            angle = self.odom_yaw + (i - count / 2.0) * (2.0 * math.pi / count) + random.uniform(-0.15, 0.15)
            candidate_x = self.odom_x + step * math.cos(angle)
            candidate_y = self.odom_y + step * math.sin(angle)
            cell = (round(candidate_x / cell_size), round(candidate_y / cell_size))
            score = self.visited_cells.get(cell, 0) + random.uniform(0.0, 0.2)
            if best is None or score < best_score:
                best = (candidate_x, candidate_y)
                best_score = score
        return best

    def compute_explore_twist(self, linear_speed: float, angular_speed: float) -> Twist:
        cell_size = float(self.get_parameter("explore_visited_cell_size_m").value)
        cell = (round(self.odom_x / cell_size), round(self.odom_y / cell_size))
        self.visited_cells[cell] = self.visited_cells.get(cell, 0) + 1

        now = time.monotonic()
        timeout = float(self.get_parameter("pose_goal_timeout_seconds").value)
        need_new_goal = self.explore_sub_goal is None or now - self.explore_sub_goal_started_at > timeout
        if not need_new_goal:
            twist, arrived = self.compute_steering_twist(
                self.explore_sub_goal[0], self.explore_sub_goal[1], linear_speed, angular_speed
            )
            if not arrived:
                return twist
        self.explore_sub_goal = self.pick_explore_subgoal()
        self.explore_sub_goal_started_at = now
        twist, _arrived = self.compute_steering_twist(
            self.explore_sub_goal[0], self.explore_sub_goal[1], linear_speed, angular_speed
        )
        return twist

    def start_manual_move(self, kind: str, modifier: str = "") -> None:
        self.manual_kind = kind
        if self.recording and kind in {"move_forward", "move_backward", "turn_left", "turn_right"}:
            is_turn = kind in {"turn_left", "turn_right"}
            duration = (
                float(self.get_parameter("manual_turn_seconds").value)
                if is_turn
                else float(self.get_parameter("manual_move_seconds").value)
            )
            self.record_buffer.append({"type": kind, "duration": duration})
        if kind in {"move_forward", "move_backward", "turn_left", "turn_right"} and not self.recording:
            # All four directions drive continuously until an explicit stop, rather
            # than a brief timed nudge - this is what keyboard teleop needs (key down
            # = move, key up = stop) and matches how an admin expects a plain drive
            # instruction to behave when typed as chat text too.
            self.manual_drive_slow = modifier == "slow"
            self.current_target = kind
            self.set_state(PatrolState.MANUAL_DRIVE)
            self.publish_event(
                "manual drive: {0}{1} (continues until stopped)".format(kind, " (slow)" if self.manual_drive_slow else "")
            )
        else:
            self.set_state(PatrolState.MANUAL)
            self.publish_event(f"manual move: {kind}")

    def adjust_speed(self, direction: str) -> None:
        step = float(self.get_parameter("speed_step").value)
        minimum = float(self.get_parameter("min_speed_scale").value)
        maximum = float(self.get_parameter("max_speed_scale").value)
        if direction == "down":
            self.speed_scale = clamp(self.speed_scale - step, minimum, maximum)
        else:
            self.speed_scale = clamp(self.speed_scale + step, minimum, maximum)
        self.publish_event("speed scale set to {0:.2f}".format(self.speed_scale))

    def request_analysis(self, question: str = "", location: str = "") -> None:
        self.pending_analysis = True
        self.pending_analysis_location = location
        self.vlm_trigger_pub.publish(String(data=json.dumps({"question": question}, ensure_ascii=False)))
        self.publish_event("requesting camera analysis" + (f" ({location})" if location else ""))

    def on_vlm_observation(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            risk = bool(payload.get("risk", False))
            summary = str(payload.get("summary", "")) or msg.data
        except (json.JSONDecodeError, AttributeError):
            summary = msg.data
            risk = any(word in msg.data.lower() for word in ["person", "hazard", "fire", "blocked"])
        self.last_vlm_summary = summary
        if self.pending_analysis:
            self.pending_analysis = False
            prefix = f"{self.pending_analysis_location}: " if self.pending_analysis_location else ""
            self.pending_analysis_location = ""
            self.publish_event(f"analysis result: {prefix}{summary[:180]}")
            if self.state == PatrolState.INSPECTING:
                # INSPECTING had no exit condition at all - once here (e.g. a
                # location with no travel path falling back to "inspect from
                # here"), the robot would rotate in place forever, since
                # nothing ever moved it out of this state. Stop as soon as the
                # single snapshot this state exists to capture is in hand.
                self.set_state(PatrolState.STOPPED)
                self.stop_motion()
        elif risk and self.state in {PatrolState.PATROLLING, PatrolState.FOLLOWING_PERSON, PatrolState.INSPECTING}:
            self.publish_event(f"attention required: {summary[:180]}")

    def on_vision_status(self, msg: String) -> None:
        self.last_vision_status = msg.data

    def on_location_feature_result(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, AttributeError):
            return
        action = payload.get("action", "")
        label = payload.get("label", "")
        if payload.get("error"):
            self.publish_event(f"location feature {action} failed for {label}: {payload['error']}")
            return
        if action == "capture":
            descriptors = payload.get("descriptors", "")
            keypoints = int(payload.get("keypoints", 0))
            if descriptors:
                self.location_store.set_features(label, descriptors, keypoints)
                self.publish_event(f"visual features captured for {label} ({keypoints} keypoints)")
            else:
                self.publish_event(
                    f"no distinct visual features found for {label} (plain scene); "
                    "location saved with move-sequence only"
                )
        elif action == "match":
            ratio = float(payload.get("match_ratio", 0.0))
            good = int(payload.get("good_matches", 0))
            keypoints = int(payload.get("keypoints", 0))
            confident = ratio >= 0.15
            verdict = "일치" if confident else "불일치 가능성 (다른 곳일 수 있음)"
            self.publish_event(
                "visual check at {0}: {1} (good matches={2}, ratio={3:.2f}, keypoints={4})".format(
                    label, verdict, good, ratio, keypoints
                )
            )

    def on_vision_cmd_vel(self, msg: Twist) -> None:
        self.last_vision_cmd = msg
        self.last_vision_cmd_at = time.monotonic()

    def drive_loop(self) -> None:
        self.tick_explore_labeling()

        linear_speed = float(self.get_parameter("linear_speed").value) * self.speed_scale
        angular_speed = float(self.get_parameter("angular_speed").value) * self.speed_scale
        use_frontier_mode = bool(self.get_parameter("explore_frontier_mode").value)

        # POSE_GOAL and EXPLORING-with-explore_frontier_mode both drive via
        # this node's own point-to-point controller (compute_steering_twist),
        # fed by visual_odom_node - not depth_nav_node/Nav2. depth_nav_node no
        # longer drives (or publishes a competing /ccai/vision_cmd_vel) during
        # POSE_GOAL or frontier-mode EXPLORING (see depth_nav_node.py's
        # explore_frontier_mode param) - it's sensor-only here, so a simple
        # stop is the safety response, not adopting its twist wholesale.
        # Adopting depth_nav_node's own full twist here used to effectively
        # hand driving authority back to its old reactive algorithm any time
        # anything was nearby - in a normal room that's almost always, so it
        # silently masked the new algorithm entirely (confirmed on real
        # hardware 2026-07-25: "no change, robot just spins in place").
        if self.state == PatrolState.POSE_GOAL:
            if self.is_obstacle_now():
                self.stop_motion()
                return
            twist = self.compute_pose_goal_twist(linear_speed, angular_speed)
            if twist is None:
                return
            self.cmd_vel_pub.publish(twist)
            return
        if self.state == PatrolState.EXPLORING and use_frontier_mode and not self.awaiting_label:
            if self.is_obstacle_now():
                # Forget the current sub-goal too, not just stop - it was
                # probably chosen straight toward whatever is now blocking us,
                # so blindly resuming toward it next tick would just repeat
                # the same stop. pick_explore_subgoal() samples several
                # candidate directions next time and should find a clearer one.
                self.explore_sub_goal = None
                self.stop_motion()
                return
            if self.has_recent_odom():
                self.cmd_vel_pub.publish(self.compute_explore_twist(linear_speed, angular_speed))
                return
            # No odometry yet (visual_odom_node still starting up, or a bad
            # stretch of frames it couldn't track) - hold still rather than
            # drive blind, safer than guessing.
            self.stop_motion()
            return

        twist = Twist()

        # MANUAL_DRIVE only ever asks vision_nav_node to drive when going forward
        # (the camera faces forward; there's nothing useful to gate on for reverse).
        vision_gated_states = {PatrolState.PATROLLING, PatrolState.FOLLOWING_PERSON}
        if self.state == PatrolState.MANUAL_DRIVE and self.manual_kind == "move_forward":
            vision_gated_states = vision_gated_states | {PatrolState.MANUAL_DRIVE}
        # Only let vision/depth drive EXPLORING while not paused for a label
        # reply - tick_explore_labeling() above already called stop_motion()
        # the moment it set awaiting_label, and excluding EXPLORING from this
        # set here means we fall through to the safe_stop_on_idle catch-all
        # below instead of a stray vision_cmd_vel overriding that stop. This
        # branch only runs at all when NOT use_frontier_mode (handled above).
        if self.state == PatrolState.EXPLORING and not self.awaiting_label:
            vision_gated_states = vision_gated_states | {PatrolState.EXPLORING}
        if self.state in vision_gated_states and self.use_recent_vision_cmd():
            self.cmd_vel_pub.publish(self.last_vision_cmd)
            return

        # Only force a safety stop once vision_nav_node has actually been contributing
        # commands and then goes stale (camera/vision lost mid-mission). If vision has
        # never published anything (disabled, not yet started, backward motion which
        # vision_nav_node doesn't drive since the camera only faces forward), fall
        # through to the plain drive pattern below instead of sitting stopped forever.
        if (
            self.state in vision_gated_states
            and bool(self.get_parameter("use_vision_cmd_vel").value)
            and self.last_vision_cmd_at > 0.0
        ):
            self.stop_motion()
            return

        if self.state == PatrolState.PATROLLING:
            elapsed = time.monotonic() - self.state_changed_at
            forward_seconds = float(self.get_parameter("patrol_forward_seconds").value)
            turn_seconds = float(self.get_parameter("patrol_turn_seconds").value)
            cycle = max(forward_seconds + turn_seconds, 0.1)
            phase = elapsed % cycle
            if phase < forward_seconds:
                # Ramp up from a crawl at the start of every forward run (including
                # right after an obstacle turn) instead of jumping straight to full
                # speed, so a lingering obstacle gets less of an impact if still close.
                twist.linear.x = linear_speed * self.ramp_factor(phase)
            else:
                twist.angular.z = angular_speed
        elif self.state == PatrolState.INSPECTING:
            # Hold still - a single VLM snapshot is captured whenever the next
            # frame arrives (see request_analysis/vlm_client_node), so rotating
            # here doesn't help it see more and previously had no exit
            # condition at all (see on_vlm_observation for where this now
            # actually ends).
            pass
        elif self.state == PatrolState.RETURNING_HOME:
            twist.linear.x = linear_speed * 0.7
            twist.angular.z = angular_speed * 0.25
        elif self.state == PatrolState.MANUAL:
            move_seconds = float(self.get_parameter("manual_move_seconds").value)
            turn_seconds = float(self.get_parameter("manual_turn_seconds").value)
            is_turn = self.manual_kind in {"turn_left", "turn_right"}
            duration = turn_seconds if is_turn else move_seconds
            manual_elapsed = time.monotonic() - self.state_changed_at
            if manual_elapsed >= duration:
                self.set_state(PatrolState.STOPPED)
                self.stop_motion()
                return
            if self.manual_kind == "move_forward":
                twist.linear.x = linear_speed * self.ramp_factor(manual_elapsed)
            elif self.manual_kind == "move_backward":
                twist.linear.x = -linear_speed * self.ramp_factor(manual_elapsed)
            elif self.manual_kind == "turn_left":
                twist.angular.z = angular_speed
            elif self.manual_kind == "turn_right":
                twist.angular.z = -angular_speed
        elif self.state == PatrolState.MANUAL_DRIVE:
            # No auto-timeout: "앞으로 가" / "천천히 앞으로 가" / keyboard-held turns keep
            # driving until an explicit stop/new direction command, per how an admin
            # actually expects a plain drive instruction to behave (not a brief safety
            # nudge) - and per how key-down/key-up teleop naturally works.
            elapsed = time.monotonic() - self.state_changed_at
            if self.manual_kind in {"move_forward", "move_backward"}:
                speed = linear_speed * self.ramp_factor(elapsed)
                if self.manual_drive_slow:
                    speed *= float(self.get_parameter("manual_drive_slow_factor").value)
                twist.linear.x = speed if self.manual_kind == "move_forward" else -speed
            elif self.manual_kind == "turn_left":
                twist.angular.z = angular_speed
            elif self.manual_kind == "turn_right":
                twist.angular.z = -angular_speed
        elif self.state == PatrolState.REPLAYING:
            if self.replay_index >= len(self.replay_steps):
                self.stop_motion()
                self.publish_event(f"arrived at {self.replay_location}")
                stored_features = self.location_store.get_features(self.replay_location)
                if stored_features:
                    self.location_feature_request_pub.publish(String(data=json.dumps(
                        {"action": "match", "label": self.replay_location, "descriptors": stored_features},
                        ensure_ascii=False,
                    )))
                self.request_analysis(self.replay_question, location=self.replay_location)
                self.set_state(PatrolState.STOPPED)
                return
            step = self.replay_steps[self.replay_index]
            step_duration = float(step.get("duration", 1.0))
            step_elapsed = time.monotonic() - self.replay_step_started_at
            if step_elapsed >= step_duration:
                self.replay_index += 1
                self.replay_step_started_at = time.monotonic()
                return
            kind = step.get("type", "")
            if kind == "move_forward":
                twist.linear.x = linear_speed * self.ramp_factor(step_elapsed)
            elif kind == "move_backward":
                twist.linear.x = -linear_speed * self.ramp_factor(step_elapsed)
            elif kind == "turn_left":
                twist.angular.z = angular_speed
            elif kind == "turn_right":
                twist.angular.z = -angular_speed
        elif bool(self.get_parameter("safe_stop_on_idle").value):
            self.stop_motion()
            return

        self.cmd_vel_pub.publish(twist)

    def ramp_factor(self, elapsed_seconds: float) -> float:
        ramp_seconds = float(self.get_parameter("speed_ramp_seconds").value)
        min_factor = float(self.get_parameter("speed_ramp_min_factor").value)
        return clamp(elapsed_seconds / max(ramp_seconds, 0.01), min_factor, 1.0)

    def use_recent_vision_cmd(self) -> bool:
        if not bool(self.get_parameter("use_vision_cmd_vel").value):
            return False
        timeout = float(self.get_parameter("vision_command_timeout_seconds").value)
        return self.last_vision_cmd_at > 0.0 and time.monotonic() - self.last_vision_cmd_at <= timeout

    def stop_motion(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def set_state(self, state: PatrolState) -> None:
        if state != self.state:
            self.state = state
            self.state_changed_at = time.monotonic()

    def publish_status(self) -> None:
        payload = {
            "node": self.get_name(),
            "state": self.state.value,
            "target": self.current_target,
            "last_vlm_summary": self.last_vlm_summary,
            "last_vision_status": self.last_vision_status,
            "patrol_elapsed_seconds": round(time.monotonic() - self.state_changed_at, 1),
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def status_text(self) -> str:
        return "status: state={0}, target={1}, last_vlm={2}".format(
            self.state.value,
            self.current_target or "none",
            (self.last_vision_status or self.last_vlm_summary or "none")[:120],
        )

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
