# 原生 Linux 操作站与树莓派部署

本目录保存已验证的有线 DDS 配置及 OLED 开机服务模板，不包含 SSH 密钥、账号密码、
运行日志或构建产物。模板约定操作站用户为 `lab`、机载用户为 `pi`，仓库位于
`~/USTROV`；使用其他账号或目录时先修改脚本和服务中的绝对路径。

## 网络与 DDS

- Linux 操作站专用有线地址：`192.168.50.1/24`。
- 树莓派 `eth0` 专用有线地址：`192.168.50.2/24`。
- 有线连接不设置默认网关；保留 WiFi 作为上网和 Windows SSH/RDP 入口。
- 两端 ROS domain 都是 `0`。Fast DDS 白名单只允许回环与上述有线地址。

在两端分别复制配置（已存在配置时先比较内容）：

```bash
mkdir -p ~/ustrov-network
cp ~/USTROV/deployment/fastdds-wired.xml ~/ustrov-network/
cp ~/USTROV/deployment/dds-wired-env.sh ~/ustrov-network/
```

加载 ROS 和工作区后，再在运行 ROS 的终端执行：

```bash
source ~/ustrov-network/dds-wired-env.sh
```

也可在各自 `.bashrc` 末尾加入同一行，让新开的交互终端自动加载。
已有 ROS 进程需要重新启动才会采用配置。修改前先上锁，不在已解锁状态中断控制链。
SSH 不会自动把操作站环境变量传到树莓派；两边都必须加载配置。

## 手动启动

操作站手柄节点使用操作站本地工作区；当前 `lab` 的安装目录是 `~/USTROV/install`。
树莓派主工作区安装目录是 `~/USTROV/ustrov_ws/install`。
启动顺序与四自由度映射见 `ustrov_remote_control/README.md`。
当前四轴增益为 `51.42`，匹配暂用的 Hippo 模型和 ±0.25 逻辑矩阵，单轴满杆的
电机命令接近 ±1；这些增益不是经过真实力臂标定的最大力或最大力矩。

## OLED

源码和接线说明见
[`ustrov_status_display`](../ustrov_ws/src/ustrov_status_display/README.md)。
屏幕使用 I2C1、`0x3c`，节点独立于 core，不发布控制命令。

首次准备驱动和编译：

```bash
bash ~/USTROV/ustrov_ws/src/ustrov_status_display/scripts/prepare_oled_deps.sh
source /opt/ros/jazzy/setup.bash
source ~/USTROV/ustrov_underlay/install/setup.bash
cd ~/USTROV/ustrov_ws
colcon build --symlink-install --packages-select ustrov_status_display
cp ~/USTROV/deployment/run-display.sh ~/ustrov-network/
```

开机服务与之前手动/tmux 启动的显示进程只能保留一个；先停止旧显示节点，再启用服务：

```bash
sudo install -m 644 ~/USTROV/deployment/ustrov-display.service /etc/systemd/system/ustrov-display.service
sudo systemctl daemon-reload
sudo systemctl enable --now ustrov-display.service
systemctl status ustrov-display.service
```

日志：`journalctl -u ustrov-display.service -f`。
此服务只启动屏幕；core/DDS 未启动时显示 `PX4 LOST` 和 `BAT --`，不自动解锁。
此前冷启动验证中，屏幕在上电约 48 秒后初始化；不要把网络等待和 ROS 环境加载时间
误判为服务没有启动。
