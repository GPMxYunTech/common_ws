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
