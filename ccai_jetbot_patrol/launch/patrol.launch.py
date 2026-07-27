import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def env_enabled(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def camera_parameters():
    mode = os.environ.get("CCAI_CAMERA_MODE", "csi").lower()
    params = {
        "camera_mode": mode,
        "use_gstreamer": mode in {"csi", "auto"},
    }
    backend = os.environ.get("CCAI_CAMERA_BACKEND")
    if backend:
        params["camera_backend"] = backend
    index = os.environ.get("CCAI_CAMERA_INDEX")
    if index:
        params["camera_index"] = int(index)
    device = os.environ.get("CCAI_CAMERA_DEVICE")
    if device:
        params["camera_device"] = device
    url = os.environ.get("CCAI_CAMERA_URL")
    if url:
        params["camera_url"] = url
    url_timeout = os.environ.get("CCAI_CAMERA_URL_TIMEOUT_SECONDS")
    if url_timeout:
        params["camera_url_timeout_seconds"] = float(url_timeout)
    retry_limit = os.environ.get("CCAI_CAMERA_RETRY_LIMIT")
    if retry_limit:
        params["max_open_attempts"] = int(retry_limit)
    retry_seconds = os.environ.get("CCAI_CAMERA_RETRY_SECONDS")
    if retry_seconds:
        params["capture_retry_seconds"] = float(retry_seconds)
    capture_width = os.environ.get("CCAI_CAMERA_CAPTURE_WIDTH")
    if capture_width:
        params["capture_width"] = int(capture_width)
    capture_height = os.environ.get("CCAI_CAMERA_CAPTURE_HEIGHT")
    if capture_height:
        params["capture_height"] = int(capture_height)
    width = os.environ.get("CCAI_CAMERA_WIDTH")
    if width:
        params["width"] = int(width)
    height = os.environ.get("CCAI_CAMERA_HEIGHT")
    if height:
        params["height"] = int(height)
    fps = os.environ.get("CCAI_CAMERA_FPS")
    if fps:
        params["fps"] = float(fps)
    jpeg_quality = os.environ.get("CCAI_CAMERA_JPEG_QUALITY")
    if jpeg_quality:
        params["jpeg_quality"] = int(jpeg_quality)
    csi_sensor_id = os.environ.get("CCAI_CSI_SENSOR_ID")
    if csi_sensor_id:
        params["csi_sensor_id"] = int(csi_sensor_id)
    csi_sensor_mode = os.environ.get("CCAI_CSI_SENSOR_MODE")
    if csi_sensor_mode:
        params["csi_sensor_mode"] = int(csi_sensor_mode)
    csi_capture_width = os.environ.get("CCAI_CSI_CAPTURE_WIDTH")
    if csi_capture_width:
        params["csi_capture_width"] = int(csi_capture_width)
    csi_capture_height = os.environ.get("CCAI_CSI_CAPTURE_HEIGHT")
    if csi_capture_height:
        params["csi_capture_height"] = int(csi_capture_height)
    csi_fps = os.environ.get("CCAI_CSI_FPS")
    if csi_fps:
        params["csi_fps"] = int(csi_fps)
    csi_flip_method = os.environ.get("CCAI_CSI_FLIP_METHOD")
    if csi_flip_method:
        params["csi_flip_method"] = int(csi_flip_method)
    reject_invalid_on_open = os.environ.get("CCAI_CAMERA_REJECT_INVALID_ON_OPEN")
    if reject_invalid_on_open:
        params["reject_invalid_on_open"] = reject_invalid_on_open.lower() in {"1", "true", "yes", "on"}
    return params


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("ccai_jetbot_patrol"), "config", "robot.yaml"])
    nodes = []
    if env_enabled("CCAI_ENABLE_HARDWARE", True):
        # respawn=True (2026-07-28): confirmed on real hardware that an
        # unhandled exception in this node (an OLED/display crash, in one
        # case) takes motor control down with it for the rest of the
        # session, since this same process owns /cmd_vel - and nothing
        # here was set up to restart it. The specific crash that triggered
        # this is fixed (see jetbot_hardware_node.py's draw_ascii_text), but
        # any *other* future fault (I2C hiccup, etc.) shouldn't be able to
        # permanently kill driving for the rest of the run either.
        nodes.append(
            Node(
                package="ccai_jetbot_patrol", executable="jetbot_hardware_node", name="jetbot_hardware_node",
                parameters=[config], output="screen", respawn=True, respawn_delay=1.0,
            )
        )
    if env_enabled("CCAI_ENABLE_CAMERA", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="camera_node", name="camera_node", parameters=[config, camera_parameters()], output="screen"))
    if env_enabled("CCAI_ENABLE_VISION", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="vision_nav_node", name="vision_nav_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_DEPTH_NAV", False):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="depth_nav_node", name="depth_nav_node", parameters=[config], output="screen"))
        if env_enabled("CCAI_ENABLE_REALSENSE_DRIVER", True):
            # Starts the D435i driver itself (realsense2_camera) as part of
            # the same launch, so depth_nav_node has something to subscribe to
            # without a separate manual `ros2 launch realsense2_camera ...`
            # command. Requires scripts/install_realsense_d435i.sh to have
            # been run first; set CCAI_ENABLE_REALSENSE_DRIVER=0 to skip this
            # and launch the realsense driver yourself instead (e.g. a
            # different resolution/rate, or it's already running elsewhere).
            nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
                ),
                launch_arguments={
                    "enable_depth": "true",
                    # Color is on now so the web UI can show the D435i's own
                    # view (with the drivable-floor overlay drawn on top) as
                    # the main preview - see depth_nav_node's color subscription.
                    "enable_color": "true",
                    "enable_infra1": "false",
                    "enable_infra2": "false",
                    "enable_gyro": "false",
                    "enable_accel": "false",
                    # NOTE: the parameter is "depth_profile", not "profile" -
                    # every log so far printed "Parameter 'depth_module.profile'
                    # is not supported" and silently ignored it, so depth was
                    # actually running at the sensor's default 848x480 this
                    # whole time instead of the requested 640x480 - almost
                    # certainly part of why the D435i started throwing
                    # "USB CAM/SCP overflow" hardware errors once both cameras
                    # (CSI + D435i) were running at once.
                    "depth_module.depth_profile": "640x480x30",
                    "rgb_camera.color_profile": "640x480x30",
                    # Needed for visual_odom_node's aligned_depth_to_color
                    # subscription (pixel-for-pixel depth+color correspondence
                    # for its ORB+Kabsch odometry). Extra CPU cost with no
                    # consumer otherwise, so only turned on when visual odom
                    # is actually enabled.
                    "align_depth.enable": "true" if env_enabled("CCAI_ENABLE_VISUAL_ODOM", False) else "false",
                }.items(),
            ))
    if env_enabled("CCAI_ENABLE_VISUAL_ODOM", False):
        # Lightweight custom RGB-D visual odometry (ORB feature matching +
        # Kabsch rigid-transform fit) feeding patrol_node's point-to-point
        # POSE_GOAL controller and coverage-seeking EXPLORING mode - see
        # visual_odom_node.py's docstring and docs/navigation_roadmap.md.
        # NOT full SLAM (no loop closure, no map, drifts over time); adopted
        # instead of rtabmap_ros/Nav2 because those packages are confirmed
        # unavailable via apt on this image (scripts/install_slam_nav2.sh)
        # and too large/risky to source-build on a 4GB Jetson Nano.
        nodes.append(Node(package="ccai_jetbot_patrol", executable="visual_odom_node", name="visual_odom_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_PATROL", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="patrol_node", name="patrol_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_VLM", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="vlm_client_node", name="vlm_client_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_EXPLORE_LLM", False):
        # Second, independent instance of the same vlm_client_node code
        # (zero code changes needed), watching the D435i's front-facing color
        # feed instead of the CSI ceiling feed the main instance above
        # watches - patrol_node's room-scan explorer (explore_room_scan_mode)
        # triggers this one specifically to ask "is there a doorway here" at
        # each rotation step. image_topic/trigger_topic are declared
        # parameters (see vlm_explore_node: block in config/robot.yaml); the
        # observation output topic is hardcoded in vlm_client_node.py, so it's
        # remapped here instead (only way to separate the two instances'
        # output without touching that file).
        nodes.append(Node(
            package="ccai_jetbot_patrol", executable="vlm_client_node", name="vlm_explore_node",
            parameters=[config],
            remappings=[("/ccai/vlm_observation", "/ccai/vlm_explore_observation")],
            output="screen",
        ))
    if env_enabled("CCAI_ENABLE_LLM", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="llm_control_node", name="llm_control_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_WEB", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="web_chat_node", name="web_chat_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_TELEGRAM", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="telegram_bridge_node", name="telegram_bridge_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_OTA", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="ota_agent_node", name="ota_agent_node", parameters=[config], output="screen"))
    return LaunchDescription(nodes)
