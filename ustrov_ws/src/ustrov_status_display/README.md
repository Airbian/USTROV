# ustrov_status_display

树莓派上的只读 SSD1306 128×64 I2C 状态屏。直接订阅 PX4 电池、飞控状态、
IMU 心跳与手动控制请求；不发布控制消息、不调用解锁或上锁服务。

## 接线

确认屏幕模块支持 3.3V 后接线：VCC→物理 Pin 1、GND→Pin 6、
SDA→Pin 3 / GPIO2、SCL→Pin 5 / GPIO3。默认使用 `/dev/i2c-1`、地址 `0x3c`。
按模块丝印核对顺序。运行用户需有 I2C 设备访问权限。

## 依赖与构建

使用系统包时安装 `python3-luma.oled python3-pil`。不便使用 sudo 时，可将
Ubuntu 官方依赖包解压到 `~/.local/share/ustrov_oled/deps/`，节点会加载其中的
`usr/lib/python3/dist-packages`；不修改系统 Python。

用户目录安装命令（只需运行一次）：

```bash
bash ~/USTROV/ustrov_ws/src/ustrov_status_display/scripts/prepare_oled_deps.sh
```

```bash
source /opt/ros/jazzy/setup.bash
source ~/USTROV/ustrov_underlay/install/setup.bash
cd ~/USTROV/ustrov_ws
colcon build --symlink-install --packages-select ustrov_status_display
source install/setup.bash
source ~/ustrov-network/dds-wired-env.sh
ros2 launch ustrov_status_display status_display.launch.py
```

此节点独立于 core；停止或拔掉屏幕不会影响推进器控制。不要同时启动多个
显示进程。屏幕断开后每 5 秒重试，默认每秒检查/刷新两次。

## 六行内容

1. 解锁状态：ARMED / DISARMED；飞控链路过期显示 UNKNOWN。
2. 电压和 PX4 `remaining` 百分比；未知值显示 `--`，不自行用电压估算电量。
3. 模式：OFFBOARD；其他模式显示 PX4 原始编号。
4. PX4 链路和 CMD 状态。CMD 需要 thrust/torque 两种输入都新鲜；未解锁
   时缺少输入为 WAIT，解锁时为 LOST。
5. eth0 的物理连接与 IPv4 地址；它不等同于操作站控制数据已到达。
6. 告警：飞控失联、Failsafe、电池过期/低电量、控制输入丢失、网线掉线或预检。

右上角的小点每秒翻转。VehicleStatus 作为状态快照缓存，IMU 数据作为持续心跳，
不依赖 TimesyncStatus。停止节点会关闭屏幕。屏幕是辅助显示，不替代飞控保护；
CPU/内核失效时屏幕可能停留旧画面，应结合右上角心跳点判断。

配置见 `config/status_display.yaml`，支持电池 ID、话题名、刷新频率、屏幕方向和
消息过期时间。多电池环境下应指定正确 `battery_id`。显示的电量准确性依赖 PX4
电池配置与电压/电流采集校准。当前仅显示主电池，不显示预计续航。

## 测试

```bash
python3 -m unittest discover -s test -v
```

测试覆盖数据过期、飞控状态缓存、未知电量和按解锁状态判断控制输入丢失。
