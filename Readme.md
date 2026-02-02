# 机械狗和无人机仿真环境建立

[![Ubuntu 20.04/22.04](https://img.shields.io/badge/Ubuntu-20.04/22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)  [![ROS Noetic](https://img.shields.io/badge/ros-noetic-brightgreen.svg?logo=ros)](https://wiki.ros.org/noetic)  [![Gazebo](https://img.shields.io/badge/Gazebo-Classic-lightgrey.svg?logo=gazebo)](http://gazebosim.org/)

本仓库提供了一种机械狗和无人机共存的仿真环境，适配宇数Go2W机械狗和PX4默认iris无人机。

## 🛠️ 环境要求
- Ubuntu 20.04, ROS1 Noetic
- PX4-Autopilot V1.17.0 alpha1
- rl_sar

## ⚙️ 快速开始
> 相关官网下载的文件会分享网盘链接
### 一、安装Ubuntu20.04
安装过程可以参考B站视频：https://www.bilibili.com/video/BV1wo4y177Gk/?spm_id_from=333.337.search-card.all.click&vd_source=9b9c8ec1527b4328f257f6cc2a8379b6
> 百度网盘中有四个版本的包，下载20.04即可
>
这里仅仅分享几个踩过的坑：
- 在空间分配方面，这里建议一共分配100G以上的空间。（若后续想要做强化学习等，需要留足空间）
- 若分为三个盘，三个盘的大小分别为：交换空间：16G |/(系统盘)：40G【不需要太多，一般文件不会放系统盘】|/home：40G以上，越多越好。不建议系统盘过大，后续文件很难从/home盘转移到/盘中。
- 设置密码越简单越好，后期下载都得输入一次密码，若复杂十分麻烦。
- 若系统提醒更新系统，一律点否。
- 安装好系统先别更新任何软件，等后续系统安装完再更新，否则容易报错。
### 二、安装ROS系统
这里直接推荐小鱼大佬的ROS安装脚本，简单好用。
安装方式可以参考Csdn网址：https://blog.csdn.net/m0_73745340/article/details/135281023
此外安装双系统的方式【若是新手，请跳过安装ros2的提示，老老实实先把ROS1安装好】：
- 在ROS官网或者小鱼安装ROS2。
- 我将我的~/.bashrc文件分享，仅供参考。里面包含了很多环境，包括选择其中的一个环境，若使用请阅读一下，删去不需要的环境再使用。
> Ubuntu20.04的好处是可以安装ROS1和ROS2。
> 
p.s 打开bashrc的方式有两个：
1. 打开Home文件夹，即点击左侧文件夹。按ctrl+h查看隐藏文件，找到.bashrc
2. （1）安装gedit修改文件
```bash
sudo apt update
sudo apt install gedit
```
2. （2）在桌面打开bashrc文件
```bash
sudo gedit ~/.bashrc
```
### 三、安装Ladder
这里安装包里给了Clash Party安装包，输入命令行安装：
命令行的开始是
```bash
sudo dpkg -i
```
后面将软件往终端拖，网址会自动弹出。比如我自己的命令行如下：
```bash
sudo dpkg -i 'sudo dpkg -i '/media/tianhaofly/Lenovo/软件/clash-party-linux-1.8.9-amd64.deb'  
```
剩下的教程直接参考赔钱机场官网，并购买相关节点
赔钱机场官网为：https://github.com/winston779/peiqianjichang
### 四、安装rl_sar
参考官网：https://github.com/fan-ziqi/rl_sar
> 所有安装步骤以该教程为准
>
将我所给go2w_description复制到src/rl_sar_zoo,对文件夹下的go2w_description进行替换
### 五、安装PX4_Autopilot V1.17.0 Alpha1
参考网址：https://gitee.com/chushengbajinban/PX4-Autopilot
#### step 1
下载PX4固件代码，并更新仓库子模块，这可能需要较长时间，请尽量挂Ladder并运行命令，请确认没有fatal错误，成功更新子模块后命令行提示应该如下图所示:
```bash
# 从github下载较新版本的px4源码 以v1.17.0-alpha1为例子
git clone -b v1.17.0-alpha1 https://github.com/PX4/PX4-Autopilot.git --recursive
```
#### step 2
运行ubuntu.sh安装所有依赖（此步骤可能耗费较长时间）
```bash
cd PX4-Autopilot/Tools/setup/
bash ubuntu.sh
```
#### step 3
重启电脑或登出用户再登入
#### step 4(Optional)
注：这一步骤可以先跳过，如果后续运行的时候没有报错，就没有必要修改。修改完了需要重新运行step 7 进行编译
注：无法确保对仿真与实机飞行是否会产生额外影响，切记。
修改PX4参数. 否则在仿真中运行OFFBOARD程序时，会报错Failsafe enabled: No manual control stick input
```bash
cd ~/PX4-Autopilot/ #改成你的路径
gedit src/modules/commander/commander_params.c
```
将PARAM_DEFINE_INT32(COM_RCL_EXCEPT, 0);修改成PARAM_DEFINE_INT32(COM_RCL_EXCEPT, 4);
#### step 6(Optional)
注：不飞实机只跑仿真就不用跑这一步骤
编译PX4固件，这其中可能会出现报错提示，请按照提示或自行百度安装相应缺少的软件/环境,请确保这行命令不会报错，再执行后续步骤:
```bash
cd ~/PX4-Autopilot/ #改成你的路径
make px4_fmu-v4
```
固件版本的选择见附录，若无特殊需求，则使用上述推荐版本
#### step 7
编译PX4仿真环境，请确保这行命令不会报错，再执行后续步骤:
```bash
DONT_RUN=1 make px4_sitl_default gazebo
# 部分版本的源码有较大改动，需要运行下面的命令才能编译
DONT_RUN=1 make px4_sitl_default gazebo-classic
```
#### step 8
将以下命令复制到~/.bashrc:
```bash
source ~/PX4_Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4_Autopilot ~/PX4_Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/rl_sar/src/rl_sar
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/rl_sar/src/rl_sar_zoo:$ROS_PACKAGE_PATH
#单机仿真运行以下命令
roslaunch px4 mavros_posix_sitl.launch
#多级仿真运行以下命令
roslaunch px4 multi_uav_mavros_sitl_sdf.launch
```
> 注意一下有的文件名叫PX4_Autopilot，有些叫PX4-Autopilot，若有区别自行修改source和export
#### step 9
MAVROS安装
```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh
```
### 六、修改PX4
#### step 1
将仓库中的mavros_posix_sitl.launch替换~/PX4_Autopilot/launch下的mavros_posix_sitl.launch
#### step 2
复制iris_with_camera到~/PX4_Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models文件夹下

## 至此环境已经全部建立，接下来打开所有的环境
### 七、打开环境
#### step 1
打开两个rl_sar，在两个文件下打开两个终端，分别输入命令行：
```bash
source devel/setup.bash
roslaunch rl_sar gazebo.launch rname:=go2w

source devel/setup.bash
rosrun rl_sar rl_sim
```
#### step 2
同样地，打开PX4：
```bash
roslaunch px4 mavros_posix_sitl.launch
```
## 环境建立完成