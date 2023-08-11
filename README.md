# SILL: Semantic Integrated LiDAR Labelling

## **Currently in development!! Do not use!! Refer to feature/for-os0-128 branch**

## Dependencies
* VisPy
* ROS
* NumPy
* SciPy

## Usage

```
rosrun sill sill.py [--start N] [--period T] [--load] path_to_dataset.bag
```
The `period` flag selects how many seconds should elapse between selected sweeps to add.
The `start` flag selets how many sweeps should be skipped to start labelling.
Note that changing `period` will affect the `start` point, since each of the `N` panos are spaced by `T`.
Setting `load` will cause the tool to look for existing labels, and if they exist will prelabel the cloud.

The dataset bag should include the following topics and can be created by recording these topics while running [ouster_decoder](https://github.com/KumarRobotics/ouster_decoder) and [llol](https://github.com/versatran01/llol).
```
/os_node/camera_info      : sensor_msgs/CameraInfo 
/os_node/image            : sensor_msgs/Image      
/os_node/llol_odom/sweep  : sensor_msgs/PointCloud2
/tf                       : tf2_msgs/TFMessage
```

The upper right corner shows the current class being labelled, the z-axis cutoff, and the index of the last shown sweep.
To load 10 more sweeps into the current map to label, press `n`.
SILL works by slicing the world in z
All points above the current elevation, shown in the upper left, are hidden.
When clicking and dragging, all points inside the circle below the selected z will be labelled the current class, but points below that have been labelled previously will not be overwritten.
To change z, Use `PgUp`/`PgDown` and press `R` to redraw at the selected height.
When `W` is pressed, labels will be written to disk in a folder in the same place as the bag file.

## Hotkey reference
- `Space`: When held, pan when clicking and dragging
- `Scroll Wheel`: Zoom
- `N`: Load 10 more sweeps
- `PgUp`/`PgDown`: Adjust the current elevation by 0.5m increments.  Note that the scene will not be redrawn
- `R`: Rerender the scene
- `O`: Enter 3D orbit mode.  Cannot label in this mode.
- `L`: Enter top-down labelling mode.
- `W`: Write labels to disk.
- `X`: Clear the current display.
- `0`-`9`: Select class


## Some labeling instructions (especially for tree turnks and light poles):
1. Quality over quantity.
2. Accuracy matters most, i.e., whatever you labeled as a tree trunk should be tree trunk, missing minor portion of tree trunks is acceptable.
3. I suggest use a small value for period, such as 0.2. Otherwise, tree trunk and light pole may look blurred and hard to label accurately. 
4. Label light pole first, then tree trunk.
5. When you are labeling light poles, increase Z till the highest point that you think your sensor FOV can reach within a reasonable distance (e.g. 10 meters), and label the whole light pole at once.
6. When you are labeling tree trunks, increase Z until you can start to see the canopy, then slightly decrease Z and you should be able to label the whole trunk at once. 
7. Combine 2D and 3D views to make sure that you labeled them correctly.
