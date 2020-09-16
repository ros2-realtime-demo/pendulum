# Instructions to test real-time capabilities

## Requirements

### Configure a real-time system

To configure a Linux real-time system please follow [these instructions](real_time_linux.md).

TODO: add support for other RTOS

## Process real-time settings

These are the options that allows to configure the process real-time settings:
* `priority`: changes the process priority and set a real-time FIFO priority. This will set the
 priority of the thread where the ROS 2 executor is running.
* `cpu-affinity`: binds the application to a specific CPU core by setting a CPU mask. For
 example, to bind the process or thread to CPU 3 (or 2 starting from 0) we set the CPU mask to 4
  (100 in binary).
* `lock-memory`: pre-faults memory until no more page faults are seen. This usually allocated a 
 high amount of memory so make sure there is enough memory in the system.
* `lock-memory-size`: specifies the amount of memory we want to pre-allocate. For example 
 `lock-memory-size 100` pre-allocates 100 MB.
* `config-child-threads`: specifies if the RMW middleware child threads will inherit the
 main process settings. This applies for `priority` and `cpu-affinity options.`. For example, if 
 `config-child-threads False` is set, only the main thread where the ROS executor is set with the 
 `priority` and `cpu-affinity` options. If, `config-child-threads True` is set, the DDS threads
  will also inherit the same priority and CPU affinity configuration than the main thread.

Example using ros2 launch: 

```bash
ros2 launch pendulum_bringup pendulum_bringup.launch.py priority:=80 cpu-affinity:=4 lock-memory-size:=100 config-child-threads:=True 
```

Example using the executable command line arguments:

```bash
ros2 run pendulum_demo pendulum_demo --priority 80 --cpu-affinity:=4 --lock-memory-size 100 --config-child-threads True 
```

### Set topic deadline QoS

TODO
