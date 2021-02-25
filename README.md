## Build

```
cd ros
catkin_make
source devel/setup.sh
```

## Run

Launch RRTstar service in a terminal session as follows:

```
roslaunch rrtstar rrtstar.launch
```

Then make calls to the service with the follow command in another terminal
session.

```
rosservice call /make_plan <startX> <startY> <goalX> <goalY>
```

Here is an example call:

```
rosservice call /make_plan 0.0 0.0 100.0 100.0
```

For more information about the service, you want to run following commands:

```
rosservice info /make_plan # Service should be running for this.

# See detailed service request and response definitions
rossrv show rrtstar/MakePlan
```

Also note that during initialization the node uses `rosparam` to get square
area side length, obstacle radius, obstacle count. Their values can be found in
the `rrtstar.launch` file.
