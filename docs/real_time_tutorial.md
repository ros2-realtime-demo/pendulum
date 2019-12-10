# Instructions to test real-time capabilities

## Requirements

### RTOS

To test the demo real-time capabilities to use a RTOS is strongly recommended but not mandatory. For Linux users you can follow [these instructions](real_time_linux.md).

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
First of all make sure you are running the demo in a RTOS.
ROS 2 uses DDS as communication middleware. Several DDS implementations are currently supported by ROS 2. In order to achieve real-time capabilities the DDS also has to be configured properly. Each DDS has it's own configuration method, usually based in a xml QoS profile file. Depending on the DDS implementation it is possible to configure the middleware threads priority, lock memory, use static memory allocation, etc.  

We will extend this section in the future providing detailed instructions about how to configure each DDS implementation. (Help is welcome!)

## Examples

For the moment it is not possible in this demo to configure the real-time settings using the ROS 2 launch system. For this reason if we want to tune the real-time settings we have to launch the demo executables using ros2 run.

### Enable statistics publishers

By default the demo won't publish any statistics with real-time information. To enable the publishing of this information we must use the `--pub-stats` option when launching the demo executable.

```
ros2 run pendulum_demo pendulum_demo  --pub-stats
```
In another terminal launch : 
```ros2 run pendulum_manager pendulum_manager```

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
```

The same for the driver simulation:

```
ros2 topic echo /driver_statistics
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
  deadline_misses_count: 0play
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

Also we can change the statistics publishing period by setting the `--stats-period` option. By default it publishes each 500 ms.

### Timer jitter measurement

Among the statistics tracker we measure the jitter of the ROS timers. For the moment we take the interval between two timer executions and we compare it with the ideal period. We take the difference and we calculate mean, min, max and standard deviation. Other methods may be applied in the future to calculate jitter ([this one for example](https://tools.ietf.org/rfcmarkup?rfc=3550&draft=&url=#page-94)).

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

We can enable memory locking with the option `--lock-memory`.The command will pre-fault memory until no more page faults are seen. This usually allocated a high amount of memory (~8GB) so make sure there is enough memory in the system.

```
ros2 run pendulum_demo pendulum_demo --lock-memory
```

Additionally, we can specify the amount of memory we want to pre-allocate with the option `--lock-memory-size`. For example we can pre-allocate 50 MB

```
ros2 run pendulum_demo pendulum_demo --lock-memory-size 50000
```

If we get the following error message this means we don't have permissions or that we can't allocate the requested amount of memory.

```
mlockall failed: Cannot allocate memory
Couldn't lock  virtual memory.
```

We can compare the difference of locking memory by comparing the number of page faults during the nodes active state.

With no memory locking we observe how the number of minor page faults increase. This is the output after one minute.

```
...
rusage_stats:
  max_resident_set_size: 42532
  total_minor_pagefaults: 7146
  total_major_pagefaults: 0
  minor_pagefaults_active_node: 3508
  major_pagefaults_active_node: 0
  voluntary_context_switches: 1048471
  involuntary_context_switches: 2947
```

Enabling memory lock, after one minute we don't observe any page fault in active state:

```
...
rusage_stats:
  max_resident_set_size: 8552112
  total_minor_pagefaults: 2132814
  total_major_pagefaults: 15
  minor_pagefaults_active_node: 0
  major_pagefaults_active_node: 0
  voluntary_context_switches: 946474
  involuntary_context_switches: 3705
```

However, notice the high amount of memory locked 8.5GB. In this we lock 100MB and we don't observe any page faults neither.

```
...
rusage_stats:
  max_resident_set_size: 264580
  total_minor_pagefaults: 60713
  total_major_pagefaults: 0
  minor_pagefaults_active_node: 0
  major_pagefaults_active_node: 0
  voluntary_context_switches: 992589
  involuntary_context_switches: 3584
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
