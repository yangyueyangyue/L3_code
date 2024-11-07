# 1. Clone
Firstly, add SSH key to gitlab, then we needn't input password any more.
follow the guide : http://172.17.51.2/-/profile/keys

```bash
$ cd ~
$ mkdir DFCV_L3
$ cd DFCV_L3
# clone source code to 'src' directory following the ROS workspace's requirement
# we use Dev_Control_Planning_Branch as main branch
$ git clone git@172.17.51.2:plan_goyu_rpo/plan_goyu_rpo -b Dev_Control_Planning_Branch src
```

# 2. Build

## 2.1. ROS
根据Ubuntu版本的不同，选择不同版本的ROS1，以Ubuntu 20.04对应的ROS noetic
### 设置ROS1镜像
参考清华镜像： https://mirrors.tuna.tsinghua.edu.cn/help/ros/
```bash
$ sudo echo 'deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main' >> /etc/apt/sources.list.d/ros-latest.list
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
$ sudo apt update
```

### Install ROS1
```bash
$ sudo apt install ros-noetic-desktop-full
```

### rosdep
参考清华镜像： https://mirrors.tuna.tsinghua.edu.cn/help/rosdistro/
1. 手动模拟 sudo rosdep init
```bash
$ sudo mkdir -p /etc/ros/rosdep/sources.list.d/
$ sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
```
2. 为 rosdep update 换源

```bash
$ echo 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' >> ~/.bashrc
$ source ~/.bashrc
$ rosdep update
```
3. 设置环境变量
```bash
$ echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
$ source ~/.bashrc

$ sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

## 2.2. 3rd party libs
Unzip the third party libs source code to ~/
```bash
$ cd ~/DFCV_L3/src
$ unzip 3rdparty.zip -d ~
```

### Eigen 3.3.0
```bash
$ cd ~/3rdparty/eigen-3.3.0/
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
```

### Gflags 2.2.2
```bash
$ cd ~/3rdparty/gflags-2.2.1/
$ mkdir build
$ cd build
$ cmake -DGFLAGS_NAMESPACE=google -DCMAKE_CXX_FLAGS=-fPIC -DBUILD_SHARED_LIBS=ON ..
$ make -j4
$ sudo make install
$ sudo ldconfig
```

### Glog 0.3.3
```bash
$ cd ~/3rdparty/glog-0.3.3/
$ ./autogen.sh
$ ./configure
$ make
$ sudo make install
```

### KVaser CAN 
```bash
$ cd ~/3rdparty/kvaser-can-5.40.102
$ sudo apt install flex
$ sudo apt install bison
$ sudo make uninstall

$ sudo apt-get install linux-headers-`uname -r`

$ make
$ sudo make install
```

// only build Canlib
```bash
$ cd canlib
$ sudo make install
```

### LCM 1.4.0
```bash
$ cd ~/3rdparty/lcm-1.4.0
$ mkdir build
$ cd build
$ cmake ..
$ sudo make install
```

### Protobuf 3.9.1
```bash
$ sudo rm -rf /usr/local/bin/protoc
$ sudo rm -rf /usr/local/include/google/protobuf
$ sudo rm -rf /usr/local/lib/libproto*

$ cd ~/3rdparty/protobuf-3.9.1
$ ./autogen.sh
$ ./configure
$ make
$ sudo make install
$ sudo ldconfig

$ sudo cp ~/3rdparty/protobuf-3.9.1/src/google/protobuf/stubs/stringprintf.h /usr/local/include/google/protobuf/stubs/
$ sudo cp ~/3rdparty/protobuf-3.9.1/src/google/protobuf/stubs/strutil.h /usr/local/include/google/protobuf/stubs/
```

when using a different protobuf version, maybe need to find all "proto_generator.sh" and execute this shell script.


## 2.3. Build L3 on PC
```bash
$ cd ~/DFCV_L3
$ catkin_make
```
When build success, we get 4 execution program (ROS1 node):

|  module  | node  | comment |
|  ----    | ----  | ----  |
| sensing  | ../devel/lib/sensing/sensing | ROS1 node and messages of sensor abstraction|
| fusion   | ../devel/lib/sensing_fusion/sensing_fusion | ROS1 node of obstacle fusion |
| planning | ../devel/lib/planning/planning | ROS1 node of behaviour and motion planning |
| control  | ../devel/lib/control/control | ROS1 node of chassis status abstraction and control |

After build finished, we copy all 4 executables to ../bin/ directory and xxx_config.xml to ../conf/ directory.

# 3. Run
In different terminals, start different nodes: 
```bash
# start ROS1 master
$ roscore
```

```bash
# start sensing node
$ cd ~/DFCV_L3/src
$ ./sensing.sh
```

```bash
# start sensor fusion node
$ cd ~/DFCV_L3/src
$ ./sensor_fusion.sh
```

```bash
# start planning node
$ cd ~/DFCV_L3/src
$ ./planning.sh
```

```bash
# start control node
$ cd ~/DFCV_L3/src
$ ./control.sh
```

# 4. VS Code
Edit, reading, analysis, debug, refactoring source code with VS Code and Extensions.
## IntelliSense
智能提示及代码跳转：
add module's include path to $folder/.vscode/c_cpp_properties.json : 
      "includePath": [
        "~/DFCV_L3/devel/include/**",
        "~/DFCV_L3/src/[planning|constrol|sensing|sensing_fusion]/src/**",
        "/opt/ros/melodic/include/**",
        "/usr/include/**"
      ]

## Debug
1. install vscode-ros plugin

2. add launch.json
see https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md

3. build with debug symbol
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo


# 5. Plotjuggler
Analysis ROS bag with Plotjuggler.
## Install
```bash
sudo apt install ros-melodic-plotjuggler*
```

## Run
```bash
rosrun plotjuggler plotjuggler
```

About how to use plotjuggler for L3, see share_disk/PNC_Team/tools/plotjuggler/plotjuggler_replay-debug.mp4

## Layout file
|  .xml   | comment  |
|  ----  | ----  |
| cc_to_acc.xml  | message and signals for CC to ACC scenario |
| curvature_dec.xml  | curve road with big curvature scenario or Lane Change function |
| follow_and_stop.xml  | ACC follow and front vehicle stop scenario |
| lca.xml  | Lane Change Assistance DDT |

## Playback
在台上可以通过在PC上回放/回灌在ADCU上录制的ROSbag数据包来分析路测中发现的问题。需要将PC上sensing, planning, control三个模块的macros.h做一些修改:
\#define ENTER_PLAYBACK_MODE (1)
\#define ENTER_PLAYBACK_MODE_ADHMI (1)

# 6. FAQ
## Open Kvaser CAN deviceds
Q : 
[can_driver_kvaser.cc:55] CANlib version 5.39
[udp_node.cc:578] UDP Receiving Thread ... [Started]
[can_driver_kvaser.cc:67] CANlib found 0 channel(s).
[can_driver_kvaser.cc:147] <-------- Open Kvaser CAN device ---------
[can_driver_kvaser.cc:155] No divece have been found, please check Kvaser CAN Device.

A : -- > reinstall kvaser driver : 
```bash
$ cd ~/3rdparty/kvaser-can-5.40.102
$ sudo make uninstall
$ sudo make install
```

# 7. Refactoring
重构是不改变功能的前提下，改善代码结构，增强代码可读性，可理解性，可扩展性。理想情况下，重构需要有足够的测试来保障，并采用小步迭代的方式。
针对速度规划代码的重构，主要改善如下几点：
- 删除废弃代码，保持代码干净
  在尝试，测试，打补丁的过程中增加了很多本应该删除的代码，导致代码的可读性差，同时可能带来误导。
  
- 功能算法可复用
  在实际的开发测试过程中发现五次多项式拟合函数的参数及状态需要根据场景做调整，将该算法进行抽象，可支持不同参数设定和预处理的通用函数，从而更灵活处理不同场景，增加代码可复用性及扩展性。
  
- 参数用变量/常量而不是魔术数字
  魔术数字通常不知道其来源，其语义，通过给其命名并增加注释增强了代码的可读性和可维护性，同时也可以根据需要标定。
  
- 代码按照场景重新组织 
对速度规划需要考虑的场景进行了抽象和归类，逻辑性更佳，可理解性及可维护性更佳，同时也利于分工合作修改和维护。

- 增加更多输出数据
当前输出的数据只有模块的最终结果，在路测中发现的问题，很难定位到更具体的代码块，通过将各种场景下计算的输入输出及中间量以ROS消息(PC)和共享数据(ADCU)的方式记录，存储，可视化，利于快速定位分析问题，尤其是偶发性问题。

