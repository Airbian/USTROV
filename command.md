# docker使用方法

docker start ros2-jazzy

docker exec -it ros2-jazzy bash

docker stop ros2-jazzy



usbipd attach --wsl Ubuntu-22.04 --busid 1-8

cd C:\\Users\\jwangma\\Documents\\ChatGPT\\UnderwaterResearch\\docker\\ros2-dev

docker compose up -d --force-recreate jazzy

docker compose exec jazzy bash



# DDS

MicroXRCEAgent serial \\

&#x20; --dev /dev/ttyPX4 \\

&#x20; -b 115200 \\

&#x20; -v 4

树莓派:

/dev/ttyAMA5



MicroXRCEAgent serial \\

&#x20; --dev /dev/fcu\_data \\

&#x20; -b 921600 \\

&#x20; -v 4

# 实机



**1.source工作区**

cd \~/USTROV/ustrov\_ws

source /opt/ros/jazzy/setup.bash

source install/setup.bash



ssh pi@ustrov.local

source /opt/ros/jazzy/setup.bash

source \~/USTROV/ustrov\_underlay/install/setup.bash

source \~/USTROV/ustrov\_ws/install/setup.bash

**2.启动core，包括DDS，执行器和传感器Bridge，Actuator Mixer**

ros2 launch ustrov\_bringup core.launch.py

**#服务，Arm**

ros2 service call \\

&#x20; /px4\_actuator\_bridge/arm \\

&#x20; std\_srvs/srv/SetBool \\

&#x20; "{data: ture}"



3\.启动EKF

ros2 launch ustrov\_state\_estimation state\_estimation.launch.py \\

&#x20; vehicle\_name:=uuv00 \\

&#x20; use\_sim\_time:=false \\

&#x20; water\_density:=1000.0 \\

&#x20; surface\_calibration\_samples:=30



## 启动仿真

### 1.source环境

source /opt/ros/jazzy/setup.bash

source \~/ros2\_underlay/install/setup.bash

source \~/ros2/install/setup.bash

source \~/ustrov\_ws/install/setup.bash

### 2.launch仿真

ros2 launch ustrov\_sim top\_hippocampus\_complete.launch.py \\

&#x20; vehicle\_name:=uuv00 \\

&#x20; start\_gui:=true \\

&#x20; fake\_state\_estimation:=false \\

&#x20; fake\_vision:=true



ros2 launch ustrov\_remote\_control keyboard\_control.launch.py \\

&#x20; vehicle\_name:=uuv00 \\

&#x20; use\_sim\_time:=true \\

&#x20; axis\_value:=0.3









坐标系



PX4 使用 FRD：

X：前

Y：右

Z：下



ROS 侧转换成 FLU：

X：前

Y：左

Z：上























start\_gazebo.launch.py：环境如何建立

&#x20;   ├── spawn\_vehicle.launch.py：机器人如何进入 Gazebo、如何接入 ROS

&#x20;   ├── spawn\_hippocampus.launch.py：HippoCampus 在通用机器人结构上增加了什么

其余文件可以分成：

spawn\_apriltag\_floor、spawn\_wall\_tags：视觉测试环境

spawn\_bluerov、top\_bluerov\_complete：另一种机器人配置



empty.sdf

&#x20;   └── 世界、物理、插件、GUI、光照



start\_gazebo.launch.py

&#x20;   ├── 加载 empty.sdf

&#x20;   ├── 生成 pool.xacro

&#x20;   └── 启动 /clock bridge



spawn\_hippocampus.launch.py

&#x20;   └── 生成 HippoCampus



HippoCampus Xacro 和 Gazebo 插件

&#x20;   └── 浮力、水动力、推进器、传感器

