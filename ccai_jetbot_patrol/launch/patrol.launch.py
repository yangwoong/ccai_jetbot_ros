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
        nodes.append(Node(package="ccai_jetbot_patrol", executable="jetbot_hardware_node", name="jetbot_hardware_node", parameters=[config], output="screen"))
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
                    # Only turned on for Nav2's costmap obstacle layer (see
                    # config/nav2_params.yaml, which reads
                    # /camera/camera/depth/color/points) - extra CPU/bandwidth
                    # cost with no consumer otherwise, since depth_nav_node
                    # reads the raw depth image directly, not a point cloud.
                    "pointcloud.enable": "true" if env_enabled("CCAI_ENABLE_NAV2", False) else "false",
                }.items(),
            ))
    if env_enabled("CCAI_ENABLE_SLAM", False):
        # RTAB-Map does RGB-D visual SLAM from the D435i alone (depth+color) -
        # no wheel encoders needed, since it computes its own visual odometry.
        # Requires scripts/install_slam_nav2.sh to have installed
        # ros-humble-rtabmap-ros first. This is a new, experimental
        # capability layered on top of - not replacing - the reactive
        # depth_nav_node patrol that remains this project's stable default;
        # see docs/navigation_roadmap.md for how they relate and what's still
        # unverified about this integration on this exact hardware/image.
        #
        # base_link -> camera static transform: the D435i is rigidly mounted
        # facing forward with no measured offset yet, so this is an identity
        # placeholder - replace with the actual mount offset (translation in
        # meters, rotation in radians as roll/pitch/yaw) once measured.
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher", name="base_link_to_camera",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "camera_link"],
        ))
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("rtabmap_launch"), "launch", "rtabmap.launch.py"])
            ),
            launch_arguments={
                "rtabmap_args": "--delete_db_on_start",
                "frame_id": "base_link",
                "depth_topic": "/camera/camera/depth/image_rect_raw",
                "rgb_topic": "/camera/camera/color/image_raw",
                "camera_info_topic": "/camera/camera/color/camera_info",
                "approx_sync": "true",
                "visual_odometry": "true",
                "qos": "2",
            }.items(),
        ))
    if env_enabled("CCAI_ENABLE_NAV2", False):
        # nav2's *navigation* launch (planner+controller+costmaps), not the
        # full *bringup* launch - bringup also starts map_server/amcl, which
        # would fight rtabmap for the map->odom transform and localization
        # role. rtabmap (above) is the map/localization source here; nav2
        # only needs to plan and drive against it. Requires CCAI_ENABLE_SLAM
        # (rtabmap providing the map + odom->base_link TF) to mean anything.
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
            ),
            launch_arguments={
                "use_sim_time": "false",
                "params_file": PathJoinSubstitution(
                    [FindPackageShare("ccai_jetbot_patrol"), "config", "nav2_params.yaml"]
                ),
            }.items(),
        ))
    if env_enabled("CCAI_ENABLE_EXPLORE_FRONTIER", False):
        # Real frontier-based exploration (explore_node.py) instead of
        # depth_nav_node's reactive "steer toward open space" heuristic during
        # autonomous exploration - see explore_node.py's docstring. Only
        # meaningful with CCAI_ENABLE_SLAM + CCAI_ENABLE_NAV2 both on; also
        # set patrol_node's explore_frontier_mode: true in robot.yaml at the
        # same time so patrol_node stops publishing its own /cmd_vel during
        # EXPLORING and fully yields to Nav2.
        nodes.append(Node(package="ccai_jetbot_patrol", executable="explore_node", name="explore_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_PATROL", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="patrol_node", name="patrol_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_VLM", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="vlm_client_node", name="vlm_client_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_LLM", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="llm_control_node", name="llm_control_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_WEB", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="web_chat_node", name="web_chat_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_TELEGRAM", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="telegram_bridge_node", name="telegram_bridge_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_OTA", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="ota_agent_node", name="ota_agent_node", parameters=[config], output="screen"))
    return LaunchDescription(nodes)
