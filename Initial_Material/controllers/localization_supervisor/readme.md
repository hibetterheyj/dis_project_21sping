## Implement supervisor
The current world provided by TA needs to be modified so that the supervisor can work
- Add a receiver node under the children node of the "DEF SUPERVISOR Robot" node
- Configure receiver parameters
    - Set receiver channel to be 1. (Normally this is the only thing you need to change, but if something's wrong you can check the following parameters)
    - Check if the emitter channel of the robot is 1.
    - BaudRate shoule be set to -1 (This means the transmission speed is infinitely fast)
    - Type should be set to "radio".
    - Type of the emitter of robot should also be "radio".
- Build both "localization_controller.c" and "localization_supervisor.c" in Webots.
- Select controller of the "DEF SUPERVISOR Robot" to be "localization_supervisor".

After finishing these things, the metric and average error will be written in "errors.csv". Set VERBOSE_err (or VERBOSE_avg_err) to be true if you want it print the error during the simulation.

