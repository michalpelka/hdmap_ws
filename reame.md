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

## Work in the container
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
