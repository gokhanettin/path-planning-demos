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
rosservice call /make_plan <squareAreaSide> <obstacleRadius> <obstacleCount> \
    <startX> <startY> <goalX> <goalY> <maxRunTime>
```

Here is an example call:

```
rosservice call /make_plan 100.0 5.0 50  0.0 0.0 100.0 100.0  2.0
```

For more information about the service, you want to run following commands:

```
rosservice info /make_plan # Service should be running for this.

# See detailed service request and response definitions
rossrv show rrtstar/MakePlan
```
