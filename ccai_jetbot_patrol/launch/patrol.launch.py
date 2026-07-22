import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def env_enabled(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def camera_parameters():
    mode = os.environ.get("CCAI_CAMERA_MODE", "usb").lower()
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
