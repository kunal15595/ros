sudo mv /opt/ros/indigo/include/urdf_parser/ /opt/ros/indigo/include/_urdf_parser/
sudo mv /opt/ros/indigo/lib/liburdfdom_model.so /opt/ros/indigo/lib/_liburdfdom_model.so
sudo mv /opt/ros/indigo/lib/liburdfdom_model_state.so /opt/ros/indigo/lib/_liburdfdom_model_state.so
sudo mv /opt/ros/indigo/lib/liburdfdom_sensor.so /opt/ros/indigo/lib/_liburdfdom_sensor.so
sudo mv /opt/ros/indigo/lib/liburdfdom_world.so /opt/ros/indigo/lib/_liburdfdom_world.so
sudo mv /opt/ros/indigo/lib/pkgconfig/urdfdom.pc /opt/ros/indigo/lib/pkgconfig/_urdfdom.pc
sudo mv /opt/ros/indigo/share/urdfdom/cmake/urdfdom-config.cmake /opt/ros/indigo/share/urdfdom/cmake/_urdfdom-config.cmake

cd ~/ros
wget https://raw.github.com/ros-gbp/urdfdom-release/debian/indigo/precise/urdfdom/package.xml
cd ~/ros/urdfdom # where the git repo was checked out
mkdir -p build
cd build
cmake ../ -DCMAKE_INSTALL_PREFIX=/opt/ros/indigo
make -j8
sudo make install

# now rebuild your catkin workspace
cd ~/ros/ros_ws
catkin_make
