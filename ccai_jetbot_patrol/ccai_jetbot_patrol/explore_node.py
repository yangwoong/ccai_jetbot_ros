import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from nav2_msgs.action import NavigateToPose
except Exception:  # pragma: no cover - only available once Nav2 is installed
    NavigateToPose = None

try:
    import tf2_ros
except Exception:  # pragma: no cover
    tf2_ros = None


class ExploreNode(Node):
    """Real frontier-based exploration: reads the occupancy grid a Visual SLAM
    map (rtabmap, see docs/navigation_roadmap.md) produces, finds the nearest
    boundary between known-free and unknown space (a "frontier"), and drives
    there via Nav2's NavigateToPose action - repeating until no frontier is
    left, i.e. the reachable area has been fully mapped.

    This replaces depth_nav_node's "steer toward whichever side looks more
    open" reactive heuristic with an actual algorithm during autonomous
    exploration - that heuristic has no notion of "unexplored" vs "already
    seen", so it can loop over the same open room forever without ever
    systematically covering it. Frontier-seeking is what actually answers
    "where haven't I been yet".

    Disabled by default (`enabled: false`) and does nothing unless a SLAM map
    (CCAI_ENABLE_SLAM) and Nav2 (CCAI_ENABLE_NAV2) are both running - see
    patrol.launch.py. When disabled, depth_nav_node's reactive patrol is
    completely unaffected - this is a separate, additive capability.
    """

    def __init__(self) -> None:
        super().__init__("explore_node")
        self.declare_parameter("enabled", False)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        # A cluster of free cells bordering unknown space smaller than this
        # (in grid cells) is treated as noise, not a real frontier to chase.
        self.declare_parameter("frontier_min_size", 8)
        # Don't re-target frontiers immediately around the robot - it just
        # came from there / is still turning to face them.
        self.declare_parameter("goal_min_distance_m", 0.5)
        # No frontier found for this long (in a row) => exploration is done.
        self.declare_parameter("no_frontier_timeout_seconds", 20.0)
        self.declare_parameter("frontier_scan_interval_seconds", 2.0)
        self.declare_parameter("nav2_action_wait_seconds", 2.0)
        self.declare_parameter("event_throttle_seconds", 15.0)

        self.np = None
        self.mode = "idle"
        self.paused = False
        self.latest_map = None
        self.goal_active = False
        self.current_goal_handle = None
        self.last_frontier_seen_at = 0.0
        self.event_throttle_at = {}
        self.tf_buffer = None
        self.tf_listener = None
        self.nav_client = None

        self.event_pub = self.create_publisher(String, "/ccai/events", 10)

        if not bool(self.get_parameter("enabled").value):
            self.get_logger().info("explore_node disabled (enable once a SLAM map + Nav2 are running)")
            return

        if NavigateToPose is None or tf2_ros is None:
            self.publish_event(
                "explore_node unavailable: nav2_msgs/tf2_ros not importable - "
                "run scripts/install_slam_nav2.sh first"
            )
            return

        try:
            import numpy as np

            self.np = np
        except Exception as exc:
            self.publish_event("explore_node unavailable: {0}".format(exc))
            return

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(OccupancyGrid, str(self.get_parameter("map_topic").value), self.on_map, 1)
        self.create_subscription(String, "/ccai/status", self.on_status, 10)
        self.create_subscription(Bool, "/ccai/explore_pause", self.on_pause, 10)
        interval = float(self.get_parameter("frontier_scan_interval_seconds").value)
        self.create_timer(interval, self.tick)
        self.last_frontier_seen_at = time.monotonic()
        self.publish_event("explore_node ready (frontier-based SLAM exploration)")

    def on_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        new_mode = str(payload.get("state", "idle"))
        if new_mode != "exploring" and self.mode == "exploring" and self.goal_active:
            self.cancel_goal()
        self.mode = new_mode

    def on_pause(self, msg: Bool) -> None:
        self.paused = bool(msg.data)
        if self.paused and self.goal_active:
            self.cancel_goal()

    def on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def tick(self) -> None:
        if self.mode != "exploring" or self.paused or self.goal_active:
            return
        if self.latest_map is None:
            return
        robot_xy = self.lookup_robot_xy()
        if robot_xy is None:
            self.publish_event_throttled(
                "explore_node: map->base_link TF not available yet (SLAM still initializing?)", key="tf"
            )
            return
        frontier = self.pick_frontier(robot_xy)
        if frontier is None:
            timeout = float(self.get_parameter("no_frontier_timeout_seconds").value)
            if time.monotonic() - self.last_frontier_seen_at > timeout:
                self.publish_event_throttled(
                    "탐색 완료: 더 이상 갈 곳(프론티어)이 없습니다 - 맵이 다 커버된 것으로 보입니다", key="done"
                )
            return
        self.last_frontier_seen_at = time.monotonic()
        self.send_goal(frontier)

    def lookup_robot_xy(self):
        try:
            global_frame = str(self.get_parameter("global_frame").value)
            robot_frame = str(self.get_parameter("robot_frame").value)
            transform = self.tf_buffer.lookup_transform(global_frame, robot_frame, rclpy.time.Time())
            return transform.transform.translation.x, transform.transform.translation.y
        except Exception as exc:
            self.get_logger().debug("tf lookup failed: {0}".format(exc))
            return None

    def pick_frontier(self, robot_xy):
        """Find the nearest large-enough cluster of free cells that border
        unknown space, in map world coordinates. Occupancy grid convention:
        -1 = unknown, 0-100 = free..occupied (threshold free at <50, matching
        Nav2's own convention for "traversable").
        """
        msg = self.latest_map
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        if width == 0 or height == 0:
            return None

        grid = self.np.array(msg.data, dtype=self.np.int16).reshape((height, width))
        free = (grid >= 0) & (grid < 50)
        unknown = grid == -1

        padded_unknown = self.np.pad(unknown, 1, mode="constant", constant_values=False)
        frontier_mask = self.np.zeros_like(free)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                shifted = padded_unknown[1 + dy: 1 + dy + height, 1 + dx: 1 + dx + width]
                frontier_mask |= free & shifted

        ys, xs = self.np.where(frontier_mask)
        if len(xs) == 0:
            return None

        # Cluster frontier cells (8-connected) with a plain BFS - fine at
        # room-scale map sizes (thousands of cells, not millions); if this
        # ever becomes a bottleneck on a much larger map, scipy.ndimage.label
        # would be the faster replacement.
        point_set = set(zip(xs.tolist(), ys.tolist()))
        visited = set()
        clusters = []
        for point in point_set:
            if point in visited:
                continue
            stack = [point]
            visited.add(point)
            cluster = []
            while stack:
                cx, cy = stack.pop()
                cluster.append((cx, cy))
                for ddx in (-1, 0, 1):
                    for ddy in (-1, 0, 1):
                        neighbor = (cx + ddx, cy + ddy)
                        if neighbor in point_set and neighbor not in visited:
                            visited.add(neighbor)
                            stack.append(neighbor)
            clusters.append(cluster)

        min_size = int(self.get_parameter("frontier_min_size").value)
        min_distance = float(self.get_parameter("goal_min_distance_m").value)
        rx, ry = robot_xy
        best = None
        best_distance = None
        for cluster in clusters:
            if len(cluster) < min_size:
                continue
            cell_x = sum(p[0] for p in cluster) / len(cluster)
            cell_y = sum(p[1] for p in cluster) / len(cluster)
            world_x = origin_x + cell_x * resolution
            world_y = origin_y + cell_y * resolution
            distance = ((world_x - rx) ** 2 + (world_y - ry) ** 2) ** 0.5
            if distance < min_distance:
                continue
            if best is None or distance < best_distance:
                best = (world_x, world_y)
                best_distance = distance
        return best

    def send_goal(self, xy) -> None:
        wait_seconds = float(self.get_parameter("nav2_action_wait_seconds").value)
        if not self.nav_client.wait_for_server(timeout_sec=wait_seconds):
            self.publish_event_throttled(
                "navigate_to_pose action server not available - is Nav2 (CCAI_ENABLE_NAV2=1) running?", key="nav2_down"
            )
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = str(self.get_parameter("global_frame").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = xy[0]
        goal.pose.pose.position.y = xy[1]
        goal.pose.pose.orientation.w = 1.0
        self.goal_active = True
        self.current_goal_handle = None
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning("navigate_to_pose goal send failed: {0}".format(exc))
            self.goal_active = False
            return
        if not goal_handle.accepted:
            self.goal_active = False
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future) -> None:
        self.goal_active = False
        self.current_goal_handle = None

    def cancel_goal(self) -> None:
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().debug("cancel_goal failed: {0}".format(exc))
        self.goal_active = False
        self.current_goal_handle = None

    def publish_event(self, text: str) -> None:
        self.event_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_event_throttled(self, text: str, key: str = "default") -> None:
        min_interval = float(self.get_parameter("event_throttle_seconds").value)
        now = time.monotonic()
        last_at = self.event_throttle_at.get(key, 0.0)
        if now - last_at < min_interval:
            return
        self.event_throttle_at[key] = now
        self.publish_event(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExploreNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
