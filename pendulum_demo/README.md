# Pendulum demo

## Memory page faults

Launch the demo without locking memory:

```
ros2 run pendulum_demo pendulum_demo --lock-memory --pub-stats
```

In other terminal show the statistics of the controller:

```
ros2 topic echo /controller_statistics
```

In other terminal activate the node:

```
ros2 lifecycle set /pendulum_controller configure
ros2 lifecycle set /pendulum_driver_node configure
ros2 lifecycle set /pendulum_controller activate
ros2 lifecycle set /pendulum_driver_node activate
```

You should see how the page number of page faults for active node increases.

Now launch again the demo locking the memory:

```
ros2 run pendulum_demo pendulum_demo --lock-memory --pub-stats
```

You should see that the number of minor page faults is zero or 1-3 and it doesn't increase.
