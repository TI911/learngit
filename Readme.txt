
/* 
* This is a test folder.
* Created by TI911, 22/023/2017, Okayama University, Okayama ,japan
*
* 
*/


Git is a version control system.
Git is free software;
Git is a distributed version control system.
lsb_release -aでバージョン確認
stretch
とのこと．なので，上記の参考をちょっと変更する必要あり．

$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu stretch main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

$ sudo apt-get update
$ sudo apt-get upgrade

sudo apt-get install cmake python-nose libgtest-dev
sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
sudo apt-get install libboost-all-dev python-empy libconsole-bridge-dev libtinyxml-dev liblz4-dev libbz2-dev

$ sudo rosdep init
$ rosdep update

$ mkdir ~/ros_catkin_ws
$ cd ~/ros_catkin_ws

$ rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall
$ wstool init src indigo-ros_comm-wet.rosinstall
$ rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:stretch
$ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo -j2
（-j2をつけないと，途中で固まる）

gtestのコンパイルで止まるので，
（/home/pi/ros_catkin_ws/build_isolated/catkin/gtest/libgtest.so"でとまる）
https://answers.ros.org/question/266665/ros-indigo-installation-problem-on-raspbian-jessie/
を参照して，以下をやる．
---
cd ~/ros_catkin_ws/src/catkin/cmake/test

and edited the gtest.cmake

$ emacs gtest.cmake

Commenting this line (#) to avoid the exclusion of this files:

if(ARG_EXCLUDE_FROM_ALL)
    #set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL TRUE)
endif()

and comment too

#set_target_properties(gtest gtest_main PROPERTIES EXCLUDE_FROM_ALL 1)

Save the file and try again

After that, the compiler compile the gtest and worked fine

----ここまで

$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
新しくターミナル立ち上げて
roscore
が動けばOK.


※追加でパッケージをいれるときは，下記のしたのほうを読む．
http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi

