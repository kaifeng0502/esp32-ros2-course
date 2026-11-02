# 安装、构建与环境边界

## 本课程验证范围

控制核心来自ESP32-S3 N16R8、ESP-IDF 5.5.6、macOS主机与Ubuntu 24.04 aarch64/ROS 2 Jazzy的实际学习实验。公开整理版做了路径、字体、默认素材和启动方式的可移植调整。硬件核心有实测记录；本公开版的可移植调整由离线测试/构建检查验证，不声称所有平台与所有接线都已实机测试。

## 1. ESP-IDF

使用[Espressif ESP32-S3入门文档](https://docs.espressif.com/projects/esp-idf/en/v5.5.6/esp32s3/get-started/index.html)，选择VS Code扩展或命令行安装。固定使用5.5.6以减少版本差异。

命令行安装的骨架如下，系统先装好官方文档列出的Git、Python、CMake等依赖：

```bash
mkdir -p "$HOME/esp"
git clone --branch v5.5.6 --recursive https://github.com/espressif/esp-idf.git "$HOME/esp/esp-idf-v5.5.6"
cd "$HOME/esp/esp-idf-v5.5.6"
./install.sh esp32s3
source export.sh
```

每个新终端都需要激活环境。不要将仓库`.venv`与ESP-IDF管理的Python环境混为一谈：`idf.py`使用IDF环境，电脑实验脚本使用装有`host/requirements.txt`的Python。

从仓库根目录构建共用固件：

```bash
cp config/sdkconfig.verified sdkconfig
./idf.sh build
```

该完整配置针对16MB Flash、160MHz、USB Serial/JTAG、当前不使用PSRAM的样机。若板子不是该配置，先核对硬件并配置，不能把16MB镜像设置无条件套给其他板子。

## 2. 端口、烧录与监控

macOS可用 `ls /dev/cu.usbmodem*`，Ubuntu可用 `ls -l /dev/ttyACM* /dev/ttyUSB*`。将实际端口写入本终端变量：

```bash
export COURSE_PORT=/dev/ttyACM0
./idf.sh -p "$COURSE_PORT" flash monitor
```

Mac通常应改为`/dev/cu.usbmodem...`。VS Code插件中也应选择实际端口和ESP32-S3。不要同时运行监控、主机实验和ROS桥接。`idf.py monitor`用Ctrl+]退出，Python/ROS程序通常用Ctrl+C。

部分Ubuntu打包的esptool缺少ESP32-S3 stub JSON，可使用发布固件包的`flash.sh`：它有SHA256校验，使用`--no-stub`的ROM烧录模式，只写bootloader、partition table和app，不写个人NVS配置或整片Flash备份。

## 3. Python主机实验

从仓库根目录：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r host/requirements.txt
python host/encoder_monitor.py --help
```

Ubuntu的ROS部分建议使用系统Python及apt依赖，避免一个没有ROS模块的虚拟环境挡住`rclpy`。不要把Mac的`.venv`复制到Ubuntu。

## 4. ROS 2 Jazzy / Ubuntu 24.04

若还没有`/opt/ros/jazzy/setup.bash`，先按[ROS 2官方Ubuntu安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)安装Jazzy。APT源配置以官方当前步骤为准，不从本课程复制过期签名密钥。

配置好ROS软件源后安装本课需要的包：

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-demo-nodes-cpp \
  ros-jazzy-rqt-plot ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
  ros-jazzy-rosidl-default-generators python3-colcon-common-extensions \
  python3-serial python3-numpy python3-matplotlib python3-pyqt5
source /opt/ros/jazzy/setup.bash
```

UTM顶部USB菜单把Espressif设备交给Ubuntu；macOS与虚拟机不能同时持有同一设备。Linux用户加入dialout后，新终端或`newgrp dialout`才获得新组权限。

```bash
sudo usermod -aG dialout "$USER"
newgrp dialout
id -nG
lsusb
```

构建：

```bash
bash scripts/ros_build.sh
source "$HOME/esp32_ros2_ws/install/setup.bash"
```

脚本只在新工作区建立源码链接并构建，不改动已有不同来源的包。需要自定义位置时执行 `bash scripts/ros_build.sh /path/to/new_ws`，之后令`COURSE_ROS_WS`指向该目录。所有相互通信的终端使用一致的`ROS_DOMAIN_ID`；离线回放用独立域。

## 5. 测试不等于实机验收

若`ros2 run`报告`No executable found`，除确认已source正确工作区外，还需检查源码可执行权限。`colcon --symlink-install`链接到原始脚本，源文件也必须可执行。本仓库已跟踪此权限，并在CI实际运行安装后的客户端验证。

```bash
python -m pip install -r tests/requirements-numerical.txt
python -m unittest discover -s tests -v
python tools/check_course.py
```

原生测试需要C编译器（macOS Command Line Tools或Linux build-essential）。SciPy只用于独立QP数值对照，不需要装到ESP32。测试不会连接真实串口、启动电机或读取私人账号。
