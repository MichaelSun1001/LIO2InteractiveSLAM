

改动如下：
1、修改了cmakelist中的内容，主要是关于jsoncpp的库的版本问题
2、在jsonconvert.h头文件中修改了#include <jsoncpp/json/json.h>
3、LIO2InteractiveSLAM的README.md有问题，所有依赖库只需要使用apt安装的库即可完成，ubuntu20（noetic）环境
4、interactive_slam的依赖库：odometry_saver有问题，在noetic下面无法编译，已解决并且上传到fork的仓库中（忘记具体什么原因了）


使用步骤：

1、FAST-LIO的bag 录制两个话题：/cloud_registered_body /Odometry
2、使用命令./run.sh将bag转成interactive_slam可以处理的两种格式：Odometry/Graph directory structure，其中Odometry这种格式需要使用rosrun interactive_slam odometry2graph命令转换成Graph格式才能被interactive_slam使用。
3、在config.json文件中修改参数，包括bag路径、odom路径、保存路径等。
4、运行interactive_slam
（巨离谱，如果使用sudo ./run.sh就会报错，不能加sudo）

非可视化运行命令：
./run.sh
可视化运行命令：
./interactive_interface.sh
