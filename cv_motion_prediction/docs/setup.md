**Connect to a Neato**

```
roslaunch neato_node bringup.launch host:=IP_ADDRESS_OF_YOUR_ROBOT
```

**Use clocks from rosbags**

```
rosparam set /use_sim_time true
```

Run rosbags with `--clock` argument.

**Uncompress Rosbag image**

```
rosrun image_transport republish in:=/camera/image_raw compressed out:=/camera/image_raw raw
```
