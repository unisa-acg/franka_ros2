# ROS 2 integration for Franka Robotics research robots

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## Tracing FCI loop timing (LTTng-UST)

`franka_hardware`'s `Robot::readOnce`/`Robot::writeOnce` wrappers are instrumented with the `franka_timing` LTTng-UST tracepoint.
Tracing needs `liblttng-ust-dev` (2.13.x) and is built by default; pass `-DWITH_LTTNG=OFF` to disable it.

Capture workflow:

```bash
lttng create franka-timing
lttng enable-event -u 'franka_timing:*'
lttng start
ros2 launch franka_bringup franka.launch.py arm_id:= fer robot_ip:=<robot-ip> load_gripper:={false|true}
lttng stop
lttng destroy
```

## License

All packages of `franka_ros2` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html

[fci-docs]: https://frankaemika.github.io/docs
