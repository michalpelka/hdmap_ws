# Preapre building
```
cd /home/robot/hdmap_ws/src/livox_ros_driver2
./build ROS1
```
build is expected to fail, but source in livox_ros_driver2 was adjusted to ROS1.
Now: 
```
cd /home/robot/hdmap_ws/
catkin_make -DROS_EDITION=ROS1
```

# Processing bag files

## Preparing an image
Update submodules:
```
git submodule update --init --recursive
```
Prepare docker image:
```
docker build -t hd_maps -f Dockerfile.desktop .
```
Now run container (using PowerShell), please adjust `D:/2022-08-19T12_49_37.772111` to your dataset.
This directory will be visible as `/data` inside a container.
```
docker run -it --rm --mount type=bind,source="D:/2022-08-19T12_49_37.772111",target=/data hd_maps
```

## Work in the container - export trajectory
Inside container process dataset to find odometry using FAST-LIO:
```
python src/rosbag_to_2000/launch.py --dir /data/ --rate 2
```
In your dataset directory `D:/2022-08-19T12_49_37.772111` the file `processed_lio.bag` should appear.

Next, export data to binary format for further processing
```
rosrun rosbag_to_2000 rosbag_livox_to_dat --dir /data/
```
After last step the `D:\2022-08-19T12_49_37.772111\data` directory should appear with exported data.

## Work in the container - export images

Inside container process dataset to export images.
```
python src/bag_to_images.py -i /data/log_*.bag -o /data/images
```
In your dataset directory `D:/2022-08-19T12_49_37.772111` the directory `images` should appear.

## Work in the container - export data in resso format

```
rosrun rosbag_to_2000 rosbag_livox_to_resso --dir /data/ --max_distance 1
```
The parameter `max_distance` allows to adjust maximum travel distance.
Running this tool results in creating the directory named `data_resso`
