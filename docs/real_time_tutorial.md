# Instructions to test real-time capabilities

First of all make sure you are running the demo in a RTOS. For Linux users you can follow [these instructions](real_time_linux.md).

## Requirements

### OSRF memory memory tools

You can use OSRF memory tools to find memory allocations in your application. To enable it
you need to do the following steps, assuming you already did compile performance test before:

1. Enter your work space: `cd pendulum_ws/src`
2. Clone OSRF memory memory tools: `git clone https://github.com/osrf/osrf_testing_tools_cpp.git`
3. Build everything `colcon build --merge-install`
4. You need to preload the memory library to make diagnostics work: `export LD_PRELOAD=$(pwd)/install/lib/libmemory_tools_interpose.so`
5. Run with memory check enabled: `ros2 run pendulum_demo pendulum_demo  --memory-check`

Note that enabling this feature will cause a huge performance impact.

### TLSF allocator

ROS 2 offers support for the TLSF (Two Level Segregate Fit) allocator, which was designed to meet real-time requirements:

* https://github.com/ros2/realtime_support/tree/master/tlsf_cpp
* https://index.ros.org/doc/ros2/Tutorials/Allocator-Template-Tutorial/

In order to use the TLSF allocator for the pendulum demo the package realtime_support must be compiled in the workspace. If it's not found we would get an error when enabling the TLSF allocator.

### DDS implementation configuration

ROS 2 uses DDS as communication middleware. Several DDS implementations are currently supported by ROS 2. In order to achieve real-time capabilities the DDS also has to be configured properly. Each DDS has it's own configuration method, usually based in a xml QoS profile file. Depending on the DDS implementation it is possible to configure the middleware threads priority, lock memory, use static memory allocation, etc.  

We will extend this section in the future providing detailed instructions about how to configure each DDS implementation. (Help is welcome!)

## Examples

For the moment it is not possible in this demo to configure the real-time setting using the ROS 2 launch system. For this reason if we want to play with the real-time settings we have to launch the demo executables using ros2 run.

### Enable statistics publishers

By default the demo won't publish any statistics with real-time information. To enable the publishing of this information we must use the `--pub-stats` option when launching the demo executable.

```
ros2 run pendulum_demo pendulum_demo  --pub-stats
```

Now we can inspect the statistics for the controller:

```
ros2 topic echo /controller_statistics
timer_stats:
  stamp:
    sec: 1571909555
    nanosec: 906944175
  timer_count: 36693
  jitter_mean_nsec: 10216.475362476946
  jitter_min_nsec: -917970.0
  jitter_max_nsec: 15299807.0
  jitter_standard_dev_nsec: 246325.80033779904
sensor_stats:
  msg_count: 36605
  deadline_misses_count: 0
command_stats:
  msg_count: 36693
  deadline_misses_count: 0
setpoint_stats:
  msg_count: 0
  deadline_misses_count: 0
rusage_stats:
  max_resident_set_size: 38340
  total_minor_pagefaults: 5874
  total_major_pagefaults: 0
  minor_pagefaults_active_node: 2031
  major_pagefaults_active_node: 0
  voluntary_context_switches: 660032
  involuntary_context_switches: 4431
---
```

The same for the driver simulation:

```
ros2 topic echo /controller_statistics
timer_stats:
  stamp:
    sec: 1571909528
    nanosec: 708351206
  timer_count: 8611
  jitter_mean_nsec: 15508.3448315912
  jitter_min_nsec: -889274.0
  jitter_max_nsec: 10080557.0
  jitter_standard_dev_nsec: 274130.4798945434
sensor_stats:
  msg_count: 8641
  deadline_misses_count: 0
command_stats:
  msg_count: 8611
  deadline_misses_count: 0
rusage_stats:
  max_resident_set_size: 37472
  total_minor_pagefaults: 4338
  total_major_pagefaults: 0
  minor_pagefaults_active_node: 493
  major_pagefaults_active_node: 0
  voluntary_context_switches: 155893
  involuntary_context_switches: 1536
---
```

Also we can change the statistics publishing period by setting the `--stats-period` option. By default it publishes each 500 ms.

### Set real-time priority

We can change the process priority and set a real-time FIFO priority. This will set the priority of the thread where the ROS 2 executor is running.

```
ros2 run pendulum_demo pendulum_demo --priority 80
```

Make sure, we have permissions to change the priority of process in the OS.

### Set CPU affinity

To bind the application to a specific CPU core we can use the option `--cpu-affinity` to set the CPU mask. For example if we want to bind the process to CPU 3 (or 2 starting from 0) we set the CPU mask to 4 (100 in binary).

```
ros2 run pendulum_demo pendulum_demo --priority 80 --cpu-affinity 4
```

### Lock memory

If our process generates memory page faults when running a real-time task we would suffer an undetermined block time. To prevent this, we must pre-allocate all the dynamic memory used by the process and lock it into RAM.

We can enable memory locking with the option `--lock-memory`. If no memory size is provided the command will pre-fault memory until no more page faults are seen. This usually allocated a high amount of memory (~8GB) so make sure there is enough memory in the system.

```
ros2 run pendulum_demo pendulum_demo --lock-memory
```

Additionally, we can specify the amount of memory we want to pre-allocate. For example we can pre-allocate 50 MB:

```
ros2 run pendulum_demo pendulum_demo --lock-memory 50000
```

### Set topic deadline QoS

For aplications requiring more control over data delivered [new DDS QoS were introduced](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html). One of this QoS is the deadline policy.

>The deadline policy establishes a contract for the amount of time allowed between messages. For Subscriptions it establishes the maximum amount of time allowed to pass between receiving messages. For Publishers it establishes the maximum amount of time allowed to pass between sending messages

In this demo, the topics between the controller and driver nodes use by default this QoS. We can configure the deadlines for publishers and subscriptions with the option `--deadline` and specifying the required deadlines in milliseconds.  

```
ros2 run pendulum_demo pendulum_demo --deadline 2
```

It is important to take into account when setting this value if intra-process communications are used or not. Latencies for intra-process communications are significantly lower than when the networking stack is used.

### TLSF allocator

It is posible to specify a custom allocator to be used by the executor. In this demo the TLSF allocator can be used with the option
`--use-tlsf`. This allocator provides bounded allocation times during runtime.

### Enable memory check

If the package `osrf_testing_tools_cpp` is available when compiling the project it is possible to enable memory checking to check if there are memory allocations during the node active state. The task execution during the nodes active state are considered in this demo as real-time compliant code. This means that there shouldn't be any memory allocation or blocking during this state. For the moment there are memory allocation in different parts of the ROS 2 libraries but in the future ideally we should see none.

For the moment we can enable memory checking with the option `-memory-check` and inspect where and when we get memory allocations.

Before enabling this option it is necessary to pre-load the library to intercept the malloc calls by calling `export LD_PRELOAD=$(pwd)/install/lib/libmemory_tools_interpose.so`. For more detailed instructions read the following tutorial:

* https://github.com/osrf/osrf_testing_tools_cpp


Note that enabling this option generates a high output print rate in the command line for short topic publish intervals. This may cause the system to become non responsive.  

### Launch standalone nodes

In the previous examples we launched a process containing both the controller and simulation/driver nodes. This means that a single executor will spin both nodes. The user may want to analyze simpler scenarios. For this reason, it is possible to launch each node in a separate process. For example we may want to launch the simulation without real-time settings and experiment just with the controller node. The same real-time options are available for each executable.

To launch each node separately we run:

In terminal 1:
```
ros2 run pendulum_demo pendulum_controller_standalone
```

In terminal 2:
```
ros2 run pendulum_demo pendulum_driver_standalone
```
