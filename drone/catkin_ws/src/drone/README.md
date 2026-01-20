# ROS package

## Running

To start connection to the flight controller, use:

```bash
roslaunch drone drone.launch
```

> Note that the package is configured to connect to `/dev/px4fmu` by default (see [previous section](#manual-installation)). Install udev rules or specify path to your FCU device in `mavros.launch`.
