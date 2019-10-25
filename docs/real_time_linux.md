# Instructions for real-time settings in Linux

## Real-time Linux

In order have real-time capabilities it is mandatory to use a RTOS. One way to achieve this is to use real-time Linux (formerly known as Linux with [preempt-rt](https://wiki.linuxfoundation.org/realtime/start) patch). In this guide you can find the required steps to setup Linux with real-time:

* https://index.ros.org/doc/ros2/Tutorials/Building-Realtime-rt_preempt-kernel-for-ROS-2/

Additionally, if we want to set the process real-time priority and lock the memory we need to configure the user permissions. Alternatively, the programs can be launched as a `sudo` user.

## Adjust permissions for memory locking

Add to `/etc/security/limits.conf` (as sudo):

```bash

<your username>    -   memlock   <limit in kB>
```

A limit of -1 is unlimited. If you choose this, you may need to accompany it with ulimit -l unlimited after editing the file.


## Setting permissions for the scheduler

Add to `/etc/security/limits.conf` (as sudo):

```bash
<your username>    -   rtprio   98
```

The range of the rtprio (real-time priority) field is 0-99. However, do NOT set the limit to 99 because then your processes could interfere with important system processes that run at the top priority (e.g. watchdog).
