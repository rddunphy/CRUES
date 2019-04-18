# CRUES
Co-operative robotics using environmental sensors


## Directory structure

* `docs`: Documentation, including PDF of final report
  * `docs/interim`: Interim report LaTeX root
  * `docs/final`: Final report LaTeX root
  * `docs/img`: Image root for reports
* `wiring`: Wiring diagrams, PCB files
* `rviz`: Rviz configuration files for visualising odemetry and mapping data
* `crues_pi`: ROS nodes and Python code
  * `crues_pi/ros_pkgs`: ROS packages to be added to catkin workspace
  * `crues_pi/crues`: Scripts used for testing and debugging
  * `crues_pi/config`: Configuration files for individual robots
       (Deploy script copies correct file to each RPi)
  * `crues_pi/crues_deploy`: Script for deploying code to RPis
  * `crues_pi/crues_run`: Script for running Roomba code on RPis


## Deploying and running code

* Turn on robots
* Connect to `CruesNet` ad-hoc WiFi network
* Add `crues_pi/crues_deploy` and `crues_pi/crues_run` to PATH
* Run `crues_deploy [ROBOTS]` to deploy code, with `[ROBOTS]` replaced by any combination of `blinky`, `inky` and `clyde`
* Run `crues_run [ROBOTS]` to run the `roomba.launch` ROS launch file

More details can be found in the [project Wiki](https://github.com/rddunphy/CRUES/wiki).

