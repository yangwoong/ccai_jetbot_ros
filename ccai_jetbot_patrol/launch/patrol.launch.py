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


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("ccai_jetbot_patrol"), "config", "robot.yaml"])
    nodes = []
    if env_enabled("CCAI_ENABLE_HARDWARE", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="jetbot_hardware_node", name="jetbot_hardware_node", parameters=[config], output="screen"))
    if env_enabled("CCAI_ENABLE_CAMERA", True):
        nodes.append(Node(package="ccai_jetbot_patrol", executable="camera_node", name="camera_node", parameters=[config], output="screen"))
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
