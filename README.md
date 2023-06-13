# common_ws

安裝所有依賴
> $ rosdep install --from-paths src --ignore-src -r -y

先編譯 msg 檔案
> $ catkin_make -DCATKIN_WHITELIST_PACKAGES="ar_track_alvar_msgs"

還原原本設定
> $ catkin_make -DCATKIN_WHITELIST_PACKAGES=""


> $ sudo apt-get install ros-melodic-serial


> $ sudo apt-get install ros-melodic-ddynamic-reconfigure


install realsense SDk
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

# Common Project Installation
```
sudo apt-get install ros-noetic-move-base ros-noetic-global-planner ros-noetic-teb-local-planner ros-noetic-map-server ros-noetic-hector-trajectory-server

mkdir -p ~/common_ws/src 

cd ~/common_ws/src

git clone https://github.com/GPMxYunTech/common_ws.git

cd ..

rosdep install --from-paths src --ignore-src -r -y

catkin_make

echo "source ~/common_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc
```
