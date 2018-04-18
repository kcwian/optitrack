# Install

* Copy package to catkin_ws/src
* Run catkin_make in catkin_ws

# How to run

## 1. Run roscore

* Open terminal and navigate to catkin_ws
* Run

```bash
source devel/setup.bash
roscore
```

## 2. Run the packag

* Open new tab in terminal in catkin_ws

```bash
source devel/setup.bash
roslaunch imu_3dm_gx4 imu.launch enable_filter:=true
```

enable_filter flag is required so that orientation information is sent to imu topic

IMPORTANT - user must be in dialout group

## 3. Echo data

Data from the device is sent to /imu/imu topic.
To see the data run:

```bash
source devel/setup.bash
rostopic echo /imu/imu
```

# ROSBAG

## Record data

```bash
source devel/setup.bash
rosbag record -O packageName /imu/imu
```

## Play data

```bash
rosbag play packageName.bag
```

--loop flag can be added to cause endless loop
