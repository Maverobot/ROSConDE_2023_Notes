
# Table of Contents

1.  [Software](#org47be03b)
2.  [Hardware](#org4d122f0)
3.  [Methods](#org0237251)
4.  [Standard](#org1bea84f)
5.  [Unsorted](#org922db8c)



<a id="org47be03b"></a>

# Software


## ROS


### [foxglove](https://foxglove.dev/)

Foxglove Studio is a powerful software tool used in robotics and automation to
visualize, debug, and analyze data from sensors and machines. It helps engineers
and developers understand how robots and systems are performing by providing a
user-friendly interface to view, analyze, and troubleshoot data collected from
various sensors and devices. Foxglove Studio plays a vital role in enhancing the
development and debugging process, allowing for more efficient and effective
robotics and automation systems.


### [ `ros2_tracing` ](https://github.com/ros2/ros2_tracing)

ros2<sub>tracing</sub> is a toolkit designed for ROS 2 (Robot Operating System 2) that
enables tracing and instrumentation within ROS 2 packages. It provides tools for
configuring tracing and offers a low-overhead framework for real-time tracing of
ROS 2 applications.


### [diagnostics](https://github.com/ros/diagnostics)

The diagnostics system in ROS (Robot Operating System) is all about gathering
and analyzing data from hardware and drivers in robots. It collects information
about devices, their status, and values, sharing it via a special topic called
`/diagnostics`.


### [ `launch_testing` ](https://github.com/ros2/launch/tree/rolling/launch_testing)

This tool is a framework for launch integration testing. For example:

-   The exit codes of all processes are available to the tests.
-   Tests can check that all processes shut down normally, or with specific exit codes.
-   Tests can fail when a process dies unexpectedly.
-   The stdout and stderr of all processes are available to the tests.
-   The command-line used to launch the processes are available to the tests.
-   Some tests run concurrently with the launch and can interact with the running processes.


### [Apex.os](https://spectrum.ieee.org/apexos-operating-system-open-source-autonomous-cars)

Apex.OS is a fork of ROS 2 that has been made robust and reliable so that it can
be used for the development and deployment of highly safety-critical systems
such as autonomous vehicles, robots, and aerospace applications. Apex.OS is
API-compatible to ROS 2. In a nutshell, Apex.OS is an SDK for autonomous
driving software and other safety-critical mobility applications. The components
enable customers to focus on building their specific applications without having
to worry about message passing, reliable real-time execution, hardware
integration, and more.


### [QGIS](https://qgis.org/en/site/)

QGIS is a user friendly Open Source Geographic Information System (GIS) licensed
under the GNU General Public License. QGIS is an official project of the Open
Source Geospatial Foundation (OSGeo). It runs on Linux, Unix, Mac OSX, Windows
and Android and supports numerous vector, raster, and database formats and
functionalities.
<https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge_demo_workspace>


### [kuberos-io](https://github.com/kuberos-io)

An ongoing project for the deployment of ROS 2 software using Kubernetes.


### [VacuSim](https://gitlab.uni-koblenz.de/intas/vacusim)

This project includes a driver for a robot vacuum cleaner as well as a
benchmarking tool, which makes it possible to gain first experiences in
programming robots using ROS 2 Humble. The scenario is an apartment that is to
be cleaned. The driver provides interfaces to actuators and sensors of the
robot. The benchmarking tool evaluates the cleaning process. The focus is on
taking the best possible route through the apartment so that the largest
possible area is covered and the floor is cleaned as well as possible.


### [Learning Robotics Fundamentals with ROS 2 and modern Gazebo](https://github.com/andreasBihlmaier/robotics_fundamentals_ros_gazebo)

This can help beginers get to understand robotics fundamentals by using ROS 2
and Gazebo.


### [ROS Team Workspace](https://github.com/StoglRobotics/ros_team_workspace)

Ros Team Workspace (RosTeamWS) is a framework for boosting collaboration in
teams when developing software for robots using Robot Operating System (ROS). It
supports both ROS and ROS 2. Its main goal is to optimize the workflow of
development teams and focus more on programming robots.


### [docker-ros](https://github.com/ika-rwth-aachen/docker-ros), [docker-ros-ml-images](https://github.com/ika-rwth-aachen/docker-ros-ml-images), [docker-run](https://github.com/ika-rwth-aachen/docker-run)

Tools, developed by Institut für Kraftfahrzeuge, RWTH Aachen, for automatically
builds development and deployment Docker images for ROS-based repositories.


### [Sick safeVisionary2](https://github.com/SICKAG/sick_safevisionary_ros2)

The official ROS2 driver for the Sick safeVisionary2 cameras.


### [robmuxinator](https://github.com/4am-robotics/robmuxinator)

The robmuxinator script serves as a command-line tool to manage and control tmux
sessions on multiple hosts of your robot. If you use `tmux` you will love
`robmuxinator`! It is designed to simplify the execution of various commands. You
can for example launch processes of your ROS bringup and application or run a
Docker container with it.


### [HelyOS](https://www.ivi.fraunhofer.de/content/dam/ivi/de/dokumente/flyer/FL_Hely_OS_Landwirtschaft_web.pdf)

Das Open-Source-Framework für Leitstände für mobile Roboter.
Related repos: <https://github.com/FraunhoferIVI>


## Other


### [sphinx-needs](https://www.sphinx-needs.com/)

Sphinx-Needs is an extension for the Python based documentation framework
Sphinx, which you can simply extend by different extensions to fulfill nearly
any requirement of a software development team.


### [Ansible](https://www.ansible.com/)

Ansible is a powerful automation tool used in IT to simplify tasks like setting
up servers, managing configurations, and deploying applications. It works by
letting you write simple scripts (playbooks) that describe the steps needed for
these tasks, making it easier to manage and scale large systems efficiently.
With Ansible, you can automate repetitive tasks and manage multiple devices or
servers from one place, saving time and reducing errors in the process.


### [perf](https://perf.wiki.kernel.org/index.php/Main_Page)

`perf` is a powerful performance analysis tool used in Linux systems to gather
detailed insights into program execution. It provides various functionalities to
measure and analyze the performance of applications and the system as a whole.
`perf` can track events such as CPU instructions, cache misses, and hardware
performance counters. It helps identify performance bottlenecks, optimize code,
and enhance overall system efficiency by offering detailed reports and metrics
for analysis.


### [flamegraps](https://github.com/brendangregg/FlameGraph)

Flame graphs are a visualization of hierarchical data, created to visualize
stack traces of profiled software so that the most frequent code-paths to be
identified quickly and accurately.


### [Xray](https://www.getxray.app/)

Xray is a comprehensive test management tool designed to streamline and enhance
software testing processes. Its primary features revolve around test planning,
execution, and reporting, aiding quality assurance teams and developers in
managing and tracking their testing activities efficiently.


### [Node-RED](https://nodered.org/)

Low-code programming for event-driven applications.


### [GTSAM](http://gtsam.org/)

GTSAM is a BSD-licensed C++ library that implements sensor fusion for
robotics and computer vision applications, including SLAM (Simultaneous
Localization and Mapping), VO (Visual Odometry), and SFM (Structure from
Motion). It uses factor graphs and Bayes networks as the underlying computing
paradigm rather than sparse matrices to optimize for the most probable
configuration or an optimal plan. Coupled with a capable sensor front-end (not
provided here), GTSAM powers many impressive autonomous systems, in both
academia and industry.


### [pybullet Industrial](https://github.com/WBK-Robotics/pybullet_industrial)

`Pybullet_industrial` is a process-aware robot simulation. It aims to enable
scientists and researchers to easily simulate robotics scenarios where a robot
is participating in a manufacturing process. It achieves this by combining the
world of classical robot simulations with the world of industrial processes. The
library is capable of simulating different manufacturing tools and workpieces,
as well as the robot itself. With the help of the pybullet<sub>industrial</sub> package
you will be able to:

-   simulate additive manufacturing processes
-   simulate milling processes and how the resulting forces impact the robot
-   simulating paint coating scenarios
-   simulate the handling of complex tasks using a variety of grippers


<a id="org4d122f0"></a>

# Hardware


## [Stereo cameras from STEREOLabs.](https://www.stereolabs.com/)


## [Robomaster S1 can be used as a robotic education kit.](https://www.dji.com/de/robomaster-s1)


## [ctrlX automation](https://apps.boschrexroth.com/microsites/ctrlx-automation/en/)


<a id="org0237251"></a>

# Methods


## 2-layer launch

Instead of a launch file structure of multiple layers, it is suggested by Ingo
Lütkebohle from BOSCH to have a 2-layer launch files

-   system launch
-   subsystem launches

This seems to have increased their productivity when dealing with launch files.


## [Agile Test Quadrants](https://testsigma.com/blog/agile-testing-quadrants/)

Agile Testing Quadrants are a practical tool for sorting testing types into four
categories. They help testers decide what to test and how to do it, considering
exhaustive testing is impossible.


## Motion planning combining Hybrid A\* and Dubins/Reeds-shepp curves

-   [Hybrid A\*](https://github.com/karlkurzer/path_planner)
-   [Dubins/Reeds-shepp curves](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-3-3-motion-planning-for-nonholonomic-mobile-robots/)


## Create own collision meshes when autogenerated model by ISacc Sim is not optimal


<a id="org1bea84f"></a>

# Standard


## ROS [REP 2004](https://ros.org/reps/rep-2004.html) - Package Quality Categrories

This REP describes a set of categories meant to convey the quality or maturity
of packages in the ROS ecosystem. Inclusion in a category, or quality level, is
based on the policies to which a package adheres. The categories address
policies about versioning, change control, documentation, testing, dependencies,
platform support and security.


## [ISO-25010](https://iso25000.com/index.php/en/iso-25000-standards/iso-25010) Software Porduct Quality

Often also useful for resolving different opinions in team.


<a id="org922db8c"></a>

# Unsorted


## "Reproducibility Safety": Make sure the binary is not being tempered after being built from the source


## Watch out for sensor timestamping instead of using Time::now() blindly

Monitor for determinism. how old are the data when being used


## ROS-ROS2 bridge is unreliable in production and "macht nur Ärger".


## BOSCH mit ROS, bzw. micro-ROS ab 2014


## Unique components -> Closed source. Others -> open source.


## Pay attention to implicit patent liscence when releasing open source software


## ISaac Sim Omnigraph


## ROS bei AGCO sein 10 Jahren

