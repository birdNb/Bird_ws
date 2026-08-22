# ROS2 环境部署

**各位大神先浏览全文档再作操作，感谢！**

概述：

文档分为电脑端操作和机器人端操作：

**电脑端负责**拉取仓库代码，编译好文件无问题后使用脚本同步到机器人上，每次运行脚本就会把本地src同步到机器人，而且会覆盖。++电脑端为非必要操作！！！++

**机器人端则**安装ros2环境及相关配置，编译从电脑端上传的src文件

**先执行电脑端操作，再执行机器人端操作！！电脑端执行过一次就行，之后的部署可以直接从机器人端操作那步骤开始！！**

**如果没有权限拉仓库代码，可以找其他方式获取ROS包，然后直接拉到机器人上编译，直接跳过电脑端操作！！！**

**本文档根据以下相关文档整合：**

[ros2安装指南](https://alidocs.dingtalk.com/i/nodes/gwva2dxOW4zEmDq5F0XZgw6PJbkz3BRL?doc_type=wiki_doc&utm_medium=dingdoc_doc_plugin_card&utm_scene=person_space&utm_source=dingdoc_doc)

[vcs代码仓库管理工具](https://alidocs.dingtalk.com/i/nodes/ydxXB52LJq19j0OkUMBYleAMJqjMp697?doc_type=wiki_doc&utm_medium=dingdoc_doc_plugin_card&utm_scene=person_space&utm_source=dingdoc_doc)

[CycloneDDS 中间件安装使用指南](https://alidocs.dingtalk.com/i/nodes/YMyQA2dXW7mERz45T5QjEjpgJzlwrZgb?doc_type=wiki_doc&utm_medium=dingdoc_doc_plugin_card&utm_scene=person_space&utm_source=dingdoc_doc)解决不同终端启动的节点之间通讯出错bug

[instinct onboard部署](https://alidocs.dingtalk.com/i/nodes/jb9Y4gmKWr1DNdXkUQ321B4L8GXn6lpz?doc_type=wiki_doc&utm_medium=dingdoc_doc_plugin_card&utm_scene=team_space&utm_source=dingdoc_doc)

[ros2 运控软件启动中心 hightorque\_bringup](https://alidocs.dingtalk.com/i/nodes/YndMj49yWjRbKjd4IDDvYbXbW3pmz5aA?cid=1593105053:1606683764&corpId=dinga276f2b8fa3092aaa1320dcb25e91351&doc_type=wiki_doc&iframeQuery=utm_medium=im_card&utm_source=im&utm_medium=im_card&utm_scene=person_space&utm_source=im)

[ros2 hightorque\_controller部署](https://alidocs.dingtalk.com/i/nodes/YMyQA2dXW7mERz45T5K44pKMJzlwrZgb?doc_type=wiki_doc&utm_medium=dingdoc_doc_plugin_card&utm_scene=person_space&utm_source=dingdoc_doc)controller为类似ros1的sim2real\_master的上层功能实现

[ros2 电机控制架构设计 hightorque\_midware](https://alidocs.dingtalk.com/i/nodes/mExel2BLV5ZenPq5FDzvm0rAJgk9rpMq?doc_type=wiki_doc&utm_medium=dingdoc_doc_plugin_card&utm_scene=person_space&utm_source=dingdoc_doc)

[ros2 电机配置说明 joints.yaml](https://alidocs.dingtalk.com/i/nodes/ZgpG2NdyVX49ng7etAGNR9jXVMwvDqPk?doc_type=wiki_doc&utm_medium=dingdoc_doc_plugin_card&utm_scene=person_space&utm_source=dingdoc_doc)

**电脑端操作：**

**1、系统安装foxy：**

```plaintext
# Set locale:
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Setup Sources:
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages:
sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-ros-base python3-argcomplete -y
sudo apt install ros-dev-tools -y

# 补齐缺少的包
sudo apt install ros-foxy-joy-linux ros-foxy-example-interfaces ros-foxy-map-msgs ros-foxy-turtlesim ros-foxy-pcl-msgs ros-foxy-gazebo-msgs  ros-foxy-rosbag2-storage-mcap  nlohmann-json3-dev python3-vcstool ros-foxy-rmw-cyclonedds-cpp ros-foxy-behaviortree-cpp-v3 libyaml-cpp-dev libglib2.0-dev libserialport-dev ros-foxy-joint-state-publisher-gui -y
```

**2、使用vcs工具拉取仓库代码**

**2.1 创建工作空间**

`vcstool` 是一个支持 svn、git、hg 和 bzr 的 VCS/SCM Python 源码控制库：

```bash
sudo apt install python3-vcstool
```

**2.2 创建工作空间**

```bash
mkdir -p ~/colcon_ws
cd ~/colcon_ws
```

**2.3 下载repos**

需要自己写策略控制逻辑的使用不带controller，希望和ros1一样的策略控制逻辑的选带controller

1.  **不带controller**
    
    一键下载  有可能需要关闭梯子，且非广州北京的同事也许需要openvpn连接公司局域网
    
    ```bash
    wget http://hightorque-install.server/download/hightorque.repos
    ```
    
    手动复制粘贴
    
    ```bash
    repositories:
      src/hightorque_bringup:
        type: git
        url: ssh://git@git.clicki.cn:22022/hightorque_motion/hightorque_bringup.git
        version: main
      src/hightorque_midware:
        type: git
        url: ssh://git@git.clicki.cn:22022/hightorque_motion/hightorque_midware.git
        version: main
      src/hightorque_msgs:
        type: git
        url: ssh://git@git.clicki.cn:22022/hightorque_motion/hightorque_msgs.git
        version: main
      src/hightorque_kinematics_core:
        type: git
        url: ssh://git@git.clicki.cn:22022/hightorque_motion/hightorque_kinematics_core.git
        version: main
      src/hightorque_kinematics_plugins:
        type: git
        url: ssh://git@git.clicki.cn:22022/hightorque_motion/hightorque_kinematics_plugins.git
        version: main
      src/hightorque_imu_ros2:
        type: git
        url: ssh://git@git.clicki.cn:22022/livelybot/hightorque_imu_ros2.git
        version: motion/feature/namespace
      src/hightorque_oled:
        type: git
        url: ssh://git@git.clicki.cn:22022/livelybot/hightorque_oled.git
        version: motion/feature/namespace
      src/hightorque_power:
        type: git
        url: ssh://git@git.clicki.cn:22022/livelybot/hightorque_power.git
        version: motion/feature/namespace
      src/hightorque_robot:
        type: git
        url: ssh://git@git.clicki.cn:22022/livelybot/hightorque_robot.git
        version: main
      src/instinct_onboard:
        type: git
        url: ssh://git@git.clicki.cn:22022/hightorque_motion/instinct_onboard.git
        version: main
    
    ```
    
2.  **带controller**
    

一键下载 有可能需要关闭梯子，且非广州北京的同事也许需要openvpn连接公司局域网

```bash
wget http://hightorque-install.server/download/hightorque_controller.repos
```

手动复制粘贴

```bash
repositories:
  src/hightorque_controller:
    type: git
    url: ssh://git@git.clicki.cn:22022/hightorque_motion/hightorque_controller.git
    version: main
  src/hightorque_bringup:
    type: git
    url: ssh://git@git.clicki.cn:22022/hightorque_motion/hightorque_bringup.git
    version: main
  src/hightorque_midware:
    type: git
    url: ssh://git@git.clicki.cn:22022/hightorque_motion/hightorque_midware.git
    version: main
  src/hightorque_msgs:
    type: git
    url: ssh://git@git.clicki.cn:22022/hightorque_motion/hightorque_msgs.git
    version: main
  src/hightorque_kinematics_core:
    type: git
    url: ssh://git@git.clicki.cn:22022/hightorque_motion/hightorque_kinematics_core.git
    version: main
  src/hightorque_kinematics_plugins:
    type: git
    url: ssh://git@git.clicki.cn:22022/hightorque_motion/hightorque_kinematics_plugins.git
    version: main
  src/hightorque_imu_ros2:
    type: git
    url: ssh://git@git.clicki.cn:22022/livelybot/hightorque_imu_ros2.git
    version: motion/feature/namespace
  src/hightorque_oled:
    type: git
    url: ssh://git@git.clicki.cn:22022/livelybot/hightorque_oled.git
    version: motion/feature/namespace
  src/hightorque_power:
    type: git
    url: ssh://git@git.clicki.cn:22022/livelybot/hightorque_power.git
    version: motion/feature/namespace
  src/hightorque_robot:
    type: git
    url: ssh://git@git.clicki.cn:22022/livelybot/hightorque_robot.git
    version: main

```

**2.4 导入所有仓库**

```bash
# 不带controller
vcs import ./ < hightorque.repos
# 带controller
vcs import ./ < hightorque_controller.repos
```

这条命令会根据清单文件将所有需要的仓库克隆到你的工作空间中。

**2.5 拉取所有仓库**

在3.4执行过导入的仓库代码后，后续直接运行下面的pull可以更新

```bash
# 不带controller
vcs pull ./ < hightorque.repos
# 带controller
vcs pull ./ < hightorque_controller.repos
```

**2.6 编译工作空间**

```bash
source /opt/ros/foxy/setup.bash
colcon build
# 或者以下编译速度更快的命令
colcon build --parallel-workers 4
```

++如果编译缺少相关的工具，按照报错的内容自行安装既可！！！++

**2.7 上传到机器人**

在本地home目录下创建bin文件夹

```shell
mkdir ~/bin
```

在.bashrc中修改环境变量

```shell
#在.bashrc加上这个，终端在任何地方都可以运行这个脚本
export PATH=$PATH:$HOME/bin
```

在~/bin下面创建s1.sh脚本,复制下面内容到此文件，

```shell
#!/bin/bash
# 机器人ip
robot="nvidia@192.168.1.60"

# 工作空间，代码拉到机器人的workspace/ros2的位置
ws="workspace/ros2"

rsync -avz --exclude='./src/.claude/' --exclude='.cache/' --exclude='.git/' --exclude='.gitignore' --exclude='log/' --exclude='build/' --exclude='install/'  src $robot:$ws/ --delete

```

给脚本添加可执行权限

```Python
chmod a+x ~/bin/s1.sh
```

**该脚本用于将本地** `**src**` **代码目录快速同步到机器人端工作空间，并排除编译缓存和版本控制文件，保证远端工程与本地代码一致。在有src的工作空间中传递。**

例如：需要把本地~/workspace/rim2real\_ros2里面的src传到机器人里面，就在~/workspace/rim2real\_ros2路径终端下直接运行  s1.sh  脚本即可。

++**每次使用脚本把代码拉到机器人端都会覆盖掉整个src！！！**++

**在电脑执行过一遍电脑端操作后，后续给不同的机器人上传代码，在~/home/bin下面继续建立s2.sh脚本、s3.sh脚本...，或者直接修改脚本里面的机器人ip就行。**

**机器人端操作：**

**1、系统安装foxy：**

直接在home下终端执行

```plaintext
# Set locale:
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Setup Sources:
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages:
sudo apt update
sudo apt upgrade  ##此步骤容易报错，需要换源
sudo apt install ros-foxy-ros-base python3-argcomplete -y
sudo apt install ros-dev-tools -y

# 补齐缺少的包
sudo apt install ros-foxy-joy-linux ros-foxy-example-interfaces ros-foxy-map-msgs ros-foxy-turtlesim ros-foxy-pcl-msgs ros-foxy-gazebo-msgs  ros-foxy-rosbag2-storage-mcap  nlohmann-json3-dev python3-vcstool ros-foxy-rmw-cyclonedds-cpp ros-foxy-behaviortree-cpp-v3 libyaml-cpp-dev libglib2.0-dev libserialport-dev ros-foxy-joint-state-publisher-gui -y
```

**2、部署编译文件**

[请至钉钉文档查看附件《ros2\_env.tar.gz》。](https://alidocs.dingtalk.com/i/nodes/YMyQA2dXW7mERz45TZXlLQK5JzlwrZgb?cid=1593105053:5780401827&corpId=dinga276f2b8fa3092aaa1320dcb25e91351&doc_type=wiki_doc&iframeQuery=anchorId%3DX02mqevg1d69o0mfllgzuu&utm_medium=im_card&utm_scene=person_space&utm_source=im)

在home目录解压这个压缩包，里面会得到三个文件夹

```plaintext
# 解压
tar -zxvf ros2_env.tar.gz
cd ~/ros2_env
# ros2_env里面有三个文件夹
# 分别在每个文件夹目录下都运行：
mkdir build && cd build
cmake ../ && make -j4 && sudo make install
```

**3、修改bashrc**

修改机器人的bashrc 

```shell
# 在bashrc里注释掉source /opt/ros/noetic/setup.bash
# 或使用命令：
sed -i 's|^source /opt/ros/noetic/setup.bash|# source /opt/ros/noetic/setup.bash|' ~/.bashrc

# 编辑bashrc
vim ~/.bashrc
按i进入编辑模式
在结尾复制粘贴下面的脚本
按esc退出编辑模式
按:wq保存退出后source ~/.bashrc

rosenv() {
    # 彻底清除所有可能的环境变量
    unset -v $(env | grep -o '^ROS[^=]*' | tr '\n' ' ')
    unset -v $(env | grep -o '^PYTHONPATH')
    unset -v $(env | grep -o '^CMAKE_PREFIX_PATH')
    unset -v $(env | grep -o '^CATKIN_INSTALL_INTO_PREFIX_ROOT')
    unset -v $(env | grep -o '^AMENT_PREFIX_PATH')
    unset -v $(env | grep -o '^COLCON_PREFIX_PATH')
    unset -v $(env | grep -o '^LD_LIBRARY_PATH')

    # 确保PATH中不包含ROS1的路径
    export PATH=$(echo $PATH | sed 's|:/opt/ros/noetic/bin||g')
    export PATH=$(echo $PATH | sed 's|/opt/ros/noetic/bin:||g')
    export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
    case $1 in
        foxy)
            source /opt/ros/foxy/setup.bash
            echo "Switched to ROS2 Foxy"
            ;;
        noetic)
            source /opt/ros/noetic/setup.bash
            echo "Switched to ROS1 Noetic"
            ;;
        *)
            echo "Usage: rosenv [foxy|noetic]"
            echo "Current ROS_DISTRO: $ROS_DISTRO"
            ;;
    esac

    # 验证环境
    echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"
    echo "PATH: $PATH"
}
```

**4、配置 ROS2 使用 CycloneDDS：**

安装 CycloneDDS RMW 实现：

```bash
sudo apt install ros-foxy-rmw-cyclonedds-cpp
```

然后在home目录下建立cyclonedds.xml文件：

```bash
#在home目录创建cyclonedds.xml文件
touch cyclonedds.xml

### 把下面内容全部粘贴到cyclonedds.xml里面
<!-- ~/cyclonedds.xml -->
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain>
    <General>
      <!-- 网卡名称 -->
      <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
      <!-- 禁止多播，禁止后无法被别的机器人或电脑发现 -->
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <!-- 列你要互相通信的机器 IP，如有 -->
        <Peer address="127.0.0.1"/>
      </Peers>
      <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```

终端输入命令，添加到 ~/.bashrc：

```bash
# 永久设置（添加到 ~/.bashrc）
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml' >> ~/.bashrc
source ~/.bashrc
```

最后：

```bash
rosenv foxy
ros2 daemon stop && ros2 daemon start
```

**5、安装miniconda**

```shell
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh

bash ~/Miniconda3-latest-Linux-aarch64.sh
# 如果运行上面一行报错，显示机器人存在miniconda3，则执行下面这一行
bash ~/Miniconda3-latest-Linux-aarch64.sh -u
# 安装过程全部选yes

source ~/.bashrc
conda deactivate
conda config --set auto_activate_base false
```

用conda python3.8环境安装，以下安装流程也是基于conda环境的

```python
conda create -n instinct_venv python=3.8
```

++**至此，ROS2的环境已经配置完毕！！！**++

++**ROS1：rosenv noetic  == source /opt/apt/ros/noetic/setup.bash**++

++**ROS2：rosenv foxy  == source /opt/apt/ros/foxy/setup.bash**++

++**下面为 编译ROS2相关功能包和运行操作！！！**++

**6、编译从电脑传入的机器人的包**

机器人上编译

```shell
rosenv foxy
#如果运行不了上面的命令，就把终端关了，重新打开运行
cd ~/workspace/ros2
colcon build
# 或者以下编译速度更快的命令
colcon build --parallel-workers 4
```

**7、instinct onboard和numpy**

将本地的instinct\_onboard上传到机器人端自己的文件夹目录下，将ros2\_numpy包下载到instinct\_onboard目录下，与README.md同级

```shell
git clone https://github.com/nitesh-subedi/ros2_numpy
```

**8、安装onnxruntime gpu**

[https://elinux.org/Jetson\_Zoo#ONNX\_Runtime](https://elinux.org/Jetson_Zoo#ONNX_Runtime)根据jetpack和cuda，python的版本

把文件传到自己的文件目录下，**在conda环境下安装**

++**只有orin的机器人要装，rk的不用，但是**++   pip install -e .这行命令仍需执行。

```shell
conda activate instinct_venv
pip install onnxruntime_gpu-1.18.0-cp38-cp38-linux_aarch64.whl
cd ~/path/to/your/src/instinct_onboard 
pip install -e . # 如果有报错就再pip install一遍

#再报错的话就运行
pip install -e . -i https://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com

pip install -e . -i https://pypi.tuna.tsinghua.edu.cn/simple 
```

[请至钉钉文档查看附件《onnxruntime\_gpu-1.18.0-cp38-cp38-linux\_aarch64.whl》。](https://alidocs.dingtalk.com/i/nodes/YMyQA2dXW7mERz45TZXlLQK5JzlwrZgb?cid=1593105053:5780401827&corpId=dinga276f2b8fa3092aaa1320dcb25e91351&doc_type=wiki_doc&iframeQuery=anchorId%3DX02mmiwd3lcrfbp95y9cpc&utm_medium=im_card&utm_scene=person_space&utm_source=im)

**9、机器人运行启动**

逻辑：

先启动电机  ---->  机器人回到零位  ---->  运行控制策略

在home目录下创建policy文件夹，将上机的policy统一上传到这里。

三步逻辑对应三个终端：

终端1，启动电机转发

```shell
cd ~/workspace/ros2/   #每台机器人cd到的位置不一样
sudo killall -9 rosmaster
rosenv foxy
source install/setup.bash
ros2 launch hightorque_bringup pi_plus_orin.launch.py
```

终端2，机器人回到零位：

```shell
cd ~/workspace/ros2   #每台机器人cd到的位置不一样
rosenv foxy
source install/setup.bash
ros2 run hightorque_midware move_to_zero.py
```

终端3，运行策略文件，需进入conda：

```shell
cd ~/workspace/ros2/   #每台机器人cd到的位置不一样
rosenv foxy
source install/setup.bash
conda activate instinct_venv
cd ~/../src/instinct_onboard/
python scripts/....
```

ros2录制rosbag：

```bash
source ~/workspace/ros2/install/setup.bash
ros2 bag record -a -s mcap -o 文件名
#或用时间戳记录
ros2 bag record -a -s mcap -o rosbag_data_$(date +%Y%m%d_%H%M%S)
```

ros2播放rosbag

```bash
cd 到rosbag包目录
ros2 bag play <rosbag包名字> --storage mcap --topics /话题1 /话题2 ..
```

```bash
## trt
/usr/src/tensorrt/bin/trtexec \
    --onnx=exported/FBcprAuxModel.onnx \
    --saveEngine=exported/FBcprAuxModel.trt

## engine
/usr/src/tensorrt/bin/trtexec \
    --onnx=exported/FBcprAuxModel.onnx \
    --saveEngine=exported/FBcprAuxModel.engine
```

**10、ros2跑ros1的动作**

终端1，启动电机转发

```shell
sudo killall rosmaster
rosenv foxy
cd ~/workspace/ros2/
source install/setup.bash
ros2 launch hightorque_bringup pi_plus_orin.launch.py
```

终端2 

```shell
cd ~/workspace/ros2
rosenv foxy
source install/setup.bash
./input.sh 
```