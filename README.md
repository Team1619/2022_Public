# 2022_Public

The uacr-framework-development project breaks the code into separate projects, each that work together to create the final robot project.

The core projects are:

- uacr-robot-core: handles common (non-hardware) logic
- frc-hardware: handles the interface to the hardware and WPILib
- uacr-robot-dashboard: creates a web browser dashboard for testing in simulation mode, on the robot, and at competitions

The 3 core projects above are used in the following robot projects:

- 2022-frc-basebot: used as a template project
- 2022-frc-compbot: 2022 competition robot
- 2022-frc-tshirtbot: off season project

To debug any of the robot projects in simulation mode:

1. Open the gradle tab
2. Go to Tasks > other
3. Right click on xyzSim (such as compbotSim)
4. Select 'Modify Run Configuration'
5. Click the 'Modify Options' dropdown box and select 'Debug all tasks on the execution graph'
6. Click 'ok'
7. Set a breakpoint
8. Double click xyzSim (such as compbotSim)
9. You should see the following messsages in the IntelliJ console window:

```
Task :2020-frc-basebot:run
    14:48:54.002 main [INFO] RobotCore - Starting services
    14:48:54.042 LoggingService [DEBUG] LoggingService - Starting LoggingService
    14:48:54.044 LoggingService [DEBUG] LoggingService - LoggingService started
    14:48:54.044 WebDashboardService [DEBUG] WebDashboardService - Starting WebDashboardService
    14:48:54.046 WebDashboardService [DEBUG] WebDashboardService - WebDashboardService started
    14:48:54.047 main [INFO] RobotCore - ********************* ALL SERVICES STARTED *******************************
    14:49:07.472 LoggingService [DEBUG] LoggingService - ********** Info thread frame cycle time = 83.0
```

10. Open a Chrome browser
11. Go to http://localhost:5800/
12. Plug in 2 Xbox Controllers to your laptop
13. Double click the web to bring a configuration screen
14. Select 'Teleop', your Xbox controllers and click 'Close'
15. You should now be able to control the code with the Xbox controllers and hit breakpoints
