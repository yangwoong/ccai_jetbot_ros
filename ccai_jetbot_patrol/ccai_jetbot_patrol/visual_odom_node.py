import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

try:
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
except Exception:  # pragma: no cover
    tf2_ros = None
    TransformStamped = None


class VisualOdomNode(Node):
    """Lightweight, self-contained RGB-D visual odometry - NOT full SLAM
    (no loop closure, no pose-graph optimization, no map). This exists
    because rtabmap_ros/Nav2 turned out to be unavailable via apt on this
    image and are too large a stack to source-build blind on a Jetson Nano
    (see docs/navigation_roadmap.md); this trades mapping/loop-closure
    accuracy for something that runs with only OpenCV, which is already
    working in this project.

    Algorithm, frame to frame:
      1. ORB keypoints+descriptors on the color frame.
      2. Match against the previous frame's keypoints (ratio test).
      3. Back-project matched pixels to 3D using the *aligned* depth frame
         (same pixel grid as color - requires align_depth.enable:=true on
         the realsense launch) and the color camera's intrinsics.
      4. Estimate the rigid transform (rotation + translation) between the
         two 3D point sets via the Kabsch algorithm (SVD) - this is exactly
         what real RGB-D odometry front-ends do before the SLAM/optimization
         part on top, we just don't have that part.
      5. Accumulate transforms into a running (x, y, yaw) pose in a fixed
         `odom` frame (wherever the robot was when this node started) and
         broadcast it as the odom->base_link TF, so the rest of the stack
         (patrol_node's point-to-point controller, location pose storage)
         can use it exactly like a "real" odometry source.

    This WILL drift over time (no correction mechanism), and fails on
    low-texture/fast-motion frames (too few good matches) - if a frame can't
    be matched well enough, this node just holds the last pose rather than
    guessing, so drift accumulates only from frames it was actually
    confident about, not from noise. Expect real error over long runs;
    good enough for "roughly get back to where I was" over one session, not
    a substitute for the accuracy a tuned real SLAM stack would give.
    """

    def __init__(self) -> None:
        super().__init__("visual_odom_node")
        self.declare_parameter("enabled", False)
        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("depth_scale_to_meters", 0.001)
        self.declare_parameter("min_valid_depth_m", 0.2)
        self.declare_parameter("max_valid_depth_m", 4.0)
        self.declare_parameter("min_good_matches", 15)
        self.declare_parameter("max_reprojection_error_m", 0.15)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # ORB detect+compute + BFMatcher + SVD on every 640x480 frame at 30fps
        # is too much for a Jetson Nano's CPU alongside camera_node/vision_nav_node/
        # depth_nav_node all running at once - process only every Nth color frame
        # to keep this from starving the rest of the stack (symptom otherwise:
        # web debug images freeze/stall system-wide, not just this node).
        self.declare_parameter("process_every_n_frames", 3)
        # Empirically flip to -1.0 if real left/right turns or forward moves
        # come out backwards once tested on the actual robot - see the
        # class docstring's note on camera-to-robot axis mapping being a
        # best-effort convention, not something verified on this hardware yet.
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("lateral_sign", 1.0)
        self.declare_parameter("forward_sign", 1.0)

        self.np = None
        self.cv2 = None
        self.orb = None
        self.matcher = None
        self.camera_info = None
        self.prev_gray = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_depth = None
        self.color_frame_count = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update_at = 0.0
        self.tf_broadcaster = None

        self.pose_pub = self.create_publisher(String, "/ccai/odom_pose", 10)
        self.event_pub = self.create_publisher(String, "/ccai/events", 10)

        if not bool(self.get_parameter("enabled").value):
            self.get_logger().info("visual_odom_node disabled")
            return

        try:
            import cv2
            import numpy as np

            self.np = np
            self.cv2 = cv2
            self.orb = cv2.ORB_create(nfeatures=400)
            self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        except Exception as exc:
            self.publish_event(f"visual_odom_node unavailable: {exc}")
            return

        if tf2_ros is None:
            self.publish_event("visual_odom_node unavailable: tf2_ros not importable")
            return
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(CameraInfo, str(self.get_parameter("camera_info_topic").value), self.on_camera_info, 1)
        self.create_subscription(Image, str(self.get_parameter("depth_topic").value), self.on_depth, 2)
        self.create_subscription(Image, str(self.get_parameter("color_topic").value), self.on_color, 2)
        self.publish_event("visual_odom_node ready (lightweight RGB-D odometry, no loop closure/map)")

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def on_depth(self, msg: Image) -> None:
        try:
            frame = self.decode_image(msg)
        except Exception:
            return
        scale = float(self.get_parameter("depth_scale_to_meters").value)
        self.latest_depth_frame = frame.astype(self.np.float32) * scale

    def decode_image(self, msg: Image):
        # Avoids a hard cv_bridge dependency for this node specifically -
        # decode straight from the raw Image message layout using numpy,
        # since we already need numpy/cv2 here regardless.
        dtype = self.np.uint16 if msg.encoding in ("16UC1", "mono16") else self.np.uint8
        channels = 1 if msg.encoding in ("16UC1", "mono16", "mono8") else 3
        array = self.np.frombuffer(bytes(msg.data), dtype=dtype)
        if channels == 1:
            return array.reshape((msg.height, msg.width))
        return array.reshape((msg.height, msg.width, channels))

    def on_color(self, msg: Image) -> None:
        if self.camera_info is None or not hasattr(self, "latest_depth_frame"):
            return
        self.color_frame_count += 1
        every_n = max(int(self.get_parameter("process_every_n_frames").value), 1)
        if self.color_frame_count % every_n != 0:
            return
        try:
            frame = self.decode_image(msg)
        except Exception:
            return
        if frame.ndim == 3 and frame.shape[2] == 3:
            gray = self.cv2.cvtColor(frame, self.cv2.COLOR_RGB2GRAY if msg.encoding == "rgb8" else self.cv2.COLOR_BGR2GRAY)
        else:
            gray = frame
        depth = self.latest_depth_frame

        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        if self.prev_gray is None or descriptors is None or self.prev_descriptors is None:
            self.prev_gray, self.prev_keypoints, self.prev_descriptors, self.prev_depth = gray, keypoints, descriptors, depth
            return

        transform = self.estimate_motion(keypoints, descriptors, depth)
        self.prev_gray, self.prev_keypoints, self.prev_descriptors, self.prev_depth = gray, keypoints, descriptors, depth
        if transform is None:
            return
        self.apply_motion(transform)
        self.publish_pose()
        self.broadcast_tf()

    def estimate_motion(self, keypoints, descriptors, depth):
        try:
            pairs = self.matcher.knnMatch(self.prev_descriptors, descriptors, k=2)
        except Exception:
            return None
        good = []
        for pair in pairs:
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < 0.75 * n.distance:
                good.append(m)

        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        min_valid = float(self.get_parameter("min_valid_depth_m").value)
        max_valid = float(self.get_parameter("max_valid_depth_m").value)

        prev_points = []
        curr_points = []
        for match in good:
            pu, pv = self.prev_keypoints[match.queryIdx].pt
            cu, cv_ = keypoints[match.trainIdx].pt
            pu_i, pv_i = int(round(pu)), int(round(pv))
            cu_i, cv_i = int(round(cu)), int(round(cv_))
            if not (0 <= pv_i < self.prev_depth.shape[0] and 0 <= pu_i < self.prev_depth.shape[1]):
                continue
            if not (0 <= cv_i < depth.shape[0] and 0 <= cu_i < depth.shape[1]):
                continue
            pz = float(self.prev_depth[pv_i, pu_i])
            cz = float(depth[cv_i, cu_i])
            if not (min_valid <= pz <= max_valid) or not (min_valid <= cz <= max_valid):
                continue
            prev_points.append(((pu_i - cx) * pz / fx, (pv_i - cy) * pz / fy, pz))
            curr_points.append(((cu_i - cx) * cz / fx, (cv_i - cy) * cz / fy, cz))

        min_matches = int(self.get_parameter("min_good_matches").value)
        if len(prev_points) < min_matches:
            return None

        prev_arr = self.np.array(prev_points)
        curr_arr = self.np.array(curr_points)
        return self.kabsch(prev_arr, curr_arr)

    def kabsch(self, prev_points, curr_points):
        """Rigid transform (R, t) mapping prev_points -> curr_points (Nx3),
        i.e. the camera's motion between the two frames."""
        prev_centroid = prev_points.mean(axis=0)
        curr_centroid = curr_points.mean(axis=0)
        prev_centered = prev_points - prev_centroid
        curr_centered = curr_points - curr_centroid
        h = prev_centered.T @ curr_centered
        u, _s, vt = self.np.linalg.svd(h)
        d = self.np.sign(self.np.linalg.det(vt.T @ u.T))
        correction = self.np.diag([1.0, 1.0, d])
        rotation = vt.T @ correction @ u.T
        translation = curr_centroid - rotation @ prev_centroid

        # Reject wildly implausible jumps (bad match set that still passed
        # the ratio test / min-match count by chance) rather than corrupt
        # the accumulated pose with one bad frame.
        max_error = float(self.get_parameter("max_reprojection_error_m").value)
        residual = curr_points - (prev_points @ rotation.T + translation)
        if float(self.np.sqrt((residual ** 2).sum(axis=1)).mean()) > max_error:
            return None
        return rotation, translation

    def apply_motion(self, transform) -> None:
        rotation, translation = transform
        # Camera optical frame convention: x=right, y=down, z=forward.
        # Robot planar convention here: x=forward, y=left, yaw=CCW about up.
        # This mapping (which camera axis is "up" vs "forward") is a
        # best-effort assumption for a level, forward-facing mount - verify
        # on the real robot and flip the *_sign params if a turn or forward
        # move comes out backwards or sideways.
        forward_sign = float(self.get_parameter("forward_sign").value)
        lateral_sign = float(self.get_parameter("lateral_sign").value)
        yaw_sign = float(self.get_parameter("yaw_sign").value)

        dx_robot = forward_sign * float(translation[2])
        dy_robot = lateral_sign * -float(translation[0])
        rvec, _ = self.cv2.Rodrigues(rotation)
        dyaw = yaw_sign * -float(rvec[1])

        cos_yaw = self.np.cos(self.yaw)
        sin_yaw = self.np.sin(self.yaw)
        self.x += dx_robot * cos_yaw - dy_robot * sin_yaw
        self.y += dx_robot * sin_yaw + dy_robot * cos_yaw
        self.yaw = self.np.arctan2(self.np.sin(self.yaw + dyaw), self.np.cos(self.yaw + dyaw))
        self.last_update_at = time.monotonic()

    def publish_pose(self) -> None:
        import json

        self.pose_pub.publish(String(data=json.dumps({"x": self.x, "y": self.y, "yaw": self.yaw}, ensure_ascii=False)))

    def broadcast_tf(self) -> None:
        if self.tf_broadcaster is None:
            return
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter("odom_frame").value)
        transform.child_frame_id = str(self.get_parameter("base_frame").value)
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = self.np.sin(self.yaw / 2.0)
        transform.transform.rotation.w = self.np.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
