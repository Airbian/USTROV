# 供交互式终端和启动脚本 source；不启动节点，也不发送控制命令。
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/ustrov-network/fastdds-wired.xml"
