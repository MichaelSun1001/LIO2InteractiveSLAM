## 0. 介绍
[interactive_slam](https://github.com/koide3/interactive_slam)是一个非常优秀的半自动地图优化工具，但是它与[hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)紧耦合，这限制了它的应用。
本工具可将任意激光雷达里程计（例如[FAST-LIO2](https://github.com/hku-mars/FAST_LIO)、[PV-LIO](https://github.com/HViktorTsoi/PV-LIO)和[Point-LIO](https://github.com/hku-mars/Point-LIO.git)等）生成的点云和轨迹数据转换为interactive_slam所需的地图格式，并且可以进行筛选关键帧、点云降采样、点云合并等预处理操作。
## 1. 依赖
### 1.1 **Ubuntu** and **ROS**
在**Ubuntu 18.04**+**ROS Melodic**的虚拟机纯净环境中测试，安装过程中可能会遇到的问题如下

### 1.2. **PCL && Eigen**
ros自带的**PCL**库即可，然后安装一些依赖
```
sudo apt-get install ros-{填入ros版本，例如melodic}-pcl-ros 
```
Eigen按照以下命令安装
```
sudo apt-get install libeigen3-dev
```

### 1.3. **GL3W && GLFW**
主要用于可视化界面

按照以下命令安装
```
sudo apt-get install libglm-dev libglfw3-dev
```
### 1.4. **g2o**
主要用于interactive_slam地图

源码安装g2o
```
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build 
cd build
cmake ../
make
sudo make install
```

#### 问题1：
安装**g2o**过程中可能会遇到cmake版本过低的问题
```
CMake 3.14 or higher is required.  You are running version 3.10.2
```
实测升级为cmake 3.20.0即可
#### 问题2：
安装**g2o**过程中可能会遇到**filesystem**头文件相关问题，实测将gcc和g++升级为gcc-9和g++-9即可
#### 问题3：
运行过程中可能会无法找到g2o部分库（例如libg2o_stuff.so），需要在文件/etc/ld.so.conf中添加以下路径
```
/usr/local/lib
```
然后运行以下命令
```
sudo ldconfig
```

## 2. 编译
创建build文件夹
```
mkdir build
```
每次编译运行脚本compile.sh
```
./compile.sh
```
## 3. 无可视化界面运行
运行脚本run.sh
```
./run.sh
```
所有参数可以在run.sh输入的./config/config.json文件中进行修改
例如
```
{
    // true代表从rosbag文件中读取里程计数据，false代表从里程计文件夹中读取里程计数据
    "from_rosbag": true,
    // rosbag路径
    "bag_path": "/xx/xxx.bag",
    // 里程计文件夹路径
    "odom_directory": "/xx/xxx",
    // 降采样分辨率
    "downsample_resolution": 0.1,
    // false代表不降采样，true代表降采样
    "is_downsample": false,
    // 关键帧距离差，单位“米”
    "keyframe_delta_x": 10,
    // 关键帧角度差,单位“度”
    "keyframe_delta_angle": 90,
    // 融合关键帧数量n,注意是前后各取n帧，所以实际融合数量为n * 2
    "friends_num": 20,
    // 0代表保存为interactive slam地图, 1代表保存为站点地图
    "save_mode": 0,
    // 存地图路径
    "save_path": "/yy/yyy"
}
```
## 4. 有可视化界面运行
运行脚本interactive_interface.sh
```
./interactive_interface.sh
```
可视化界面具体操作见wiki
