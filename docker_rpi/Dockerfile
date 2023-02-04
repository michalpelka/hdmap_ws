# docker build -t livox -f Dockerfile . --cpuset-cpus="0" --memory="1G"
# rocker --x11 --nvidia --network host livox

# set host ip to 192.168.1.50
# assume Livox ip is 192.168.1.134

# inside container:
# ping 192.168.1.177
# vim /ws/src/livox_ros_driver2/config/MID360_config.json 
# screen -m bash -c '. /ws/devel/setup.sh && roslaunch /ws/src/livox_ros_driver2/launch_ROS1/msg_MID360.launch'
# . /ws/devel/setup.sh && . /ws/devel/setup.sh && roslaunch /ws/src/FAST_LIO/launch/mapping_avia.launch 


ARG ROS_DISTRO=melodic

FROM ros:$ROS_DISTRO-ros-core-bionic

RUN apt-get update && apt-get install -y \
      git build-essential \
      ros-melodic-perception-pcl \
#      ros-melodic-rviz \ 
      ros-melodic-eigen-conversions \
      ros-melodic-compressed-image-transport \
      vim iputils-ping screen && \
    rm -rf /var/lib/apt/lists/*

## Livox SDK2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    mkdir -p ./Livox-SDK2/build && \
    cd ./Livox-SDK2/build && \
    cmake .. && make && make install
    
# clone hdmapping    
RUN . /opt/ros/melodic/setup.sh && \
    git clone https://github.com/michalpelka/hdmap_ws -b livox2&& \
    cd /hdmap_ws/ && \
    git submodule update --init --recursive 


RUN touch /hdmap_ws/src/pointgrey_camera_driver/CATKIN_IGNORE && \
    touch /hdmap_ws/src/ublox/CATKIN_IGNORE && \
    touch /hdmap_ws/src/rosbag_to_2000/CATKIN_IGNORE && \
    touch /hdmap_ws/src/FAST_LIO/CATKIN_IGNORE

RUN . /opt/ros/melodic/setup.sh && \
    cd /hdmap_ws/src/livox_ros_driver2 && ./build.sh ROS1

# RUN . /opt/ros/melodic/setup.sh && \
#     cd /ws/src && \
#     git clone https://github.com/michalpelka/FAST_LIO.git -b livox_sdk2 && \
#     cd FAST_LIO && git submodule init && git submodule update

# RUN . /opt/ros/melodic/setup.sh && \
#     cd /ws && catkin_make
    
RUN sed -i 's/192.168.1.12/192.168.1.134/g' /hdmap_ws/src/livox_ros_driver2/config/MID360_config.json
RUN sed -i 's/192.168.1.5/192.168.1.50/g' /hdmap_ws/src/livox_ros_driver2/config/MID360_config.json



RUN echo "source /opt/ros/melodic/setup.bash\nsource /hdmap_ws/devel/setup.bash\nexport HD_Repo=/data/\nroslaunch hd_mapping system.launch\n" > /entry_point.sh 
RUN chmod +x /entry_point.sh

ENTRYPOINT ["/bin/bash","-c","/entry_point.sh"]
