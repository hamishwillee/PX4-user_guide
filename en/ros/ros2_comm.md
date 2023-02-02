# ROS 2 User Guide

The ROS2-PX4 architecture provides a deep integration between ROS 2 and PX4, allowing ROS 2 subscribers or publisher nodes to interface directly with PX4 uORB topics.

This topic provides an overview of the architecture and application pipeline, and explains how to setup and use ROS 2 with PX4.

## Overview

The application pipeline for ROS 2 is very straightforward, thanks to the use of the _XRCE-DDS_ communications middleware.

![Architecture XRCE-DDS with ROS 2](../../assets/middleware/xrce_dds/architecture_xrce-dds_ros2.svg)

<!-- doc source: https://docs.google.com/drawings/d/1WcJOU-EcVOZRPQwNzMEKJecShii2G4U3yhA3U6C4EhE/edit?usp=sharing -->

The XRCE-DDS middleware consists of a client running on PX4 and an agent running on the companion computer, with bi-directional data exchange between them over a serial, UDP, TCP or custom link.
The agent acts as a proxy for the client to publish and subscribe to topics in the global DDS data space.

The PX4 [microdds-client](../modules/modules_system.md#microdds-client) is generated at build time and included in PX4 firmare by default.
It includes both the "generic" XRCE-DDS client code, and translation code that it uses to publish to/from uORB topics.
The subset of uORB messages that are generated into the client are listed in [PX4-Autopilot/src/modules/microdds_client/dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/microdds_client/dds_topics.yaml).
The generator uses the uORB message definitions in the source tree: [PX4-Autopilot/msg](https://github.com/PX4/PX4-Autopilot/tree/main/msg) to create the message definitions.

ROS 2 needs to have the _same_ message definitions that were used to create the XRCE-DDS client module in the PX4 Firmware.
When working with PX4 main and normal releases you can clone the [PX4/px4_msgs](https://github.com/PX4/px4_msgs) package, which contains the definitions for each release in separate branches.

Note that the XRCE-DDS _agent_ itself has no dependency on client code and can be built from [source](https://github.com/eProsima/Micro-XRCE-DDS-Agent) as part of a ROS 2 build, or installed as a snap (in this document we build from source as part of our workspace).

The last thing to note is that the XRCE-DDS client is built into firmware by default but not started automatically except for simulator builds.
Therefore you will normally need to start both the client and agent when using ROS 2.


## Installation & Setup

The supported platform for PX4 development is Ubuntu 20.04 (at time of writing), which means that you should use ROS2 "Foxy".

:::warning
Other platforms, such as Ubuntu 22.04 and ROS 2 "Humble", may appear to work, but are not supported by the PX4 dev team. <!-- Windows/Mac? -->
:::

To setup ROS 2 for use with PX4 you will need to:

- [Install ROS2](#install-ros-2)
- [Build ROS 2 Workspace](#build-ros-2-workspace)
- [Sanity Check the Installation](#sanity-check-the-installation) (Optional)

### Install ROS 2

To install ROS 2 and its dependencies:

1. [Install ROS 2 Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)
   - You can install _either_ the desktop (`ros-foxy-desktop`) or bare-bones (`ros-foxy-ros-base`) version
   - You should additionally install the development tools (`ros-dev-tools`)
1. Some Python dependencies must also be installed (using **`pip`** or **`apt`**):

   ```sh
   sudo pip3 install -U empy pyros-genmsg setuptools
   ```

### Setup Agent & Simulator Client

https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html
sudo snap install micro-xrce-dds-agent
sudo snap install micro-xrce-dds-agent --edge
micro-xrce-dds-agent udp4 -p 8888


```sh
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

> The eProsima Micro XRCE-DDS Agent can be configured at compile-time via several CMake definitions. Find them listed in the Configuration section of the eProsima Micro XRCE-DDS Agent page.

Now the the executable eProsima Micro XRCE-DDS Agent is installed in the system. Before running it, add /usr/local/lib to the dynamic loader-linker directories.

1. Start the agent with settings for connecting to the XRCE-DDS client running on the simulator: 

   ```sh
   MicroXRCEAgent udp4 -p 8888
   ```



### Build ROS 2 Workspace

This example demonstrates how to build and run the XRCE-DDS agent and [px4_msgs](https://github.com/PX4/px4_msgs) in a ROS 2 workspace.
It assumes the source code is hosted in your home directory (modify the commands as needed to put the source code somewhere else).

Note that the agent code does not actually _use_ the messages.
That are included for demonstration, because almost all ROS2 code for PX4 _will_ need the [px4_msgs](https://github.com/PX4/px4_msgs) to be built in the same workspace!

:::note
The agent can also be built outside of ROS and installed, or installed from a package.
For more information see: [XRCE-DDS (PX4-ROS2/DDS Bridge) > ????](../middleware/xrce_dds.md#TBDTHISLINK).
:::

#### Building the Workspace

To create and build the workspace:

1. Create a workspace directory using:
   ```sh
   mkdir -p ~/px4_ros_xrce_dds_ws/src
   ```
   :::note
   A naming convention for workspace folders can make it easier to manage workspaces.
   :::
1. Clone the source code for the [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) and[px4_msgs](https://github.com/PX4/px4_msgs) to the `/src` directory (the `main` branch is cloned by default):

   ```sh
   cd ~/px4_ros_xrce_dds_ws/src
   git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
   git clone https://github.com/PX4/px4_msgs.git
   ```
1. Source the "foxy" development environment and compile the workspace using `colcon`:

   ```sh
   cd ..
   source /opt/ros/foxy/setup.bash
   colcon build
   ```
   This builds all the folders under `/src` using the "foxy" toolchain.


#### Start the Agent

To run the executables in a workspace with ROS 2 you need to source `local_setup.bash`.
This provides access to the "environment hooks" for the current workspace.
In other words, it makes the executables that were just built available in the current terminal.

To run the XRCE-DDS agent in the workspace:

1. Source the `local_setup.bash` (also `setup.bash` if in a new terminal).

   ```sh
   source /opt/ros/foxy/setup.bash
   source install/local_setup.bash
   ```
1. Start the agent with settings for connecting to the XRCE-DDS client running on the simulator: 

   ```sh
   MicroXRCEAgent udp4 -p 8888
   ```

   For information about connecting to real hardware see: [XRCE-DDS (PX4-ROS2/DDS Bridge) > ????](../middleware/xrce_dds.md#TBDTHISLINK).
   
The agent is now running, but you won't be seeing much unless it is connected to a client that is publishing ROS topics (which we'll set up in the next step).

:::note
You can leave the agent running and use if for other sections!
Note that only one agent is allowed per connection channel.
:::

#### Start the Client

To test everthing is working we'll run the PX4 simulator, which publishes ROS topics by default to the port on which the agent is listening.

The steps are:

1. Set up a PX4 development environment in the normal way:
   - [Setup the development environment for Ubuntu](../dev_setup/dev_env_linux_ubuntu.md)
   - [Download PX4 source](../dev_setup/building_px4.md)

1. Open a new terminal in the root of the **PX4 Autopilot** project, and then start a PX4 [Gazebo Classic](../sim_gazebo_classic/README.md) simulation using:

   ```sh
   make px4_sitl gazebo-classic
   ```

   Once PX4 has fully started the terminal will display the [NuttShell/System Console](../debug/system_console.md).
   
   After the agent connects this should include `INFO` messages showing creation of data writers, like this:
   ```
   ...
   INFO  [microdds_client] synchronized with time offset 1675929429203524us
   INFO  [microdds_client] successfully created rt/fmu/out/failsafe_flags data writer, topic id: 83
   INFO  [microdds_client] successfully created rt/fmu/out/sensor_combined data writer, topic id: 168
   INFO  [microdds_client] successfully created rt/fmu/out/timesync_status data writer, topic id: 188
   ...
   ```
   
   The agent will also start showing output as equivalent topics are created in the DDS network:
   
   ```
   ...
   [1675929445.268957] info     | ProxyClient.cpp    | create_publisher         | publisher created      | client_key: 0x00000001, publisher_id: 0x0DA(3), participant_id: 0x001(1)
   [1675929445.269521] info     | ProxyClient.cpp    | create_datawriter        | datawriter created     | client_key: 0x00000001, datawriter_id: 0x0DA(5), publisher_id: 0x0DA(3)
   [1675929445.270412] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x00000001, topic_id: 0x0DF(2), participant_id: 0x001(1)
   ...
   ```

Then let's have a look at some of the published topics:

1. Open a new terminal, navigate to our workspace, and source ROS 2 and the workspace environment:

   ```
   cd ~/px4_ros_xrce_dds_ws
   source /opt/ros/foxy/setup.bash
   source install/local_setup.bash
   ```
1. Use `ros2 topic list` to list the topics visible to ROS 2:

   ```sh
   ros2 topic list
   ```
   
   The result will be a list of topic types:
   ```
   /fmu/in/obstacle_distance
   /fmu/in/offboard_control_mode
   /fmu/in/onboard_computer_status
   ...
   ```

2. Use `ros2 topic echo` to show the details of a particular topic `/fmu/out/vehicle_status`.

   :::note
   This command only works because we sourced `local_setup.bash`, which contains the build definitions of `px4_msgs`.
   Without these, the `ros2` environment knows the message types, but not the details.
   :::

   ```she
   ros2 topic echo /fmu/out/vehicle_status
   ```
   
   The command will echo the topic details as they update.
   ```
   ---
   timestamp: 1675931593364359
   armed_time: 0
   takeoff_time: 0
   arming_state: 1
   latest_arming_reason: 0
   latest_disarming_reason: 0
   nav_state_timestamp: 3296000
   nav_state_user_intention: 4
   nav_state: 4
   failure_detector_status: 0
   hil_state: 0
   ...
   ---
   ```

## ROS 2 Example Applications

### ROS 2 Listener

The ROS 2 listener example demonstrates how to write ROS nodes to listen to topics published by PX4.
The code observes just one topic ([sensor_combined](../msg_docs/sensor_combined.md#sensor-combined-uorb-message)), but the same pattern can generally be used to monitor PX4 telmetry.

The example source code is in the [px4_ros_com](https://github.com/PX4/px4_ros_com) repo under `/src/examples/listeners`. 

:::note
[px4_ros_com](https://github.com/PX4/px4_ros_com) is not a dependency for using PX4 and ROS2, but contains some useful examples.
:::

#### Trying it out

First let's try the example out. 
We do this in much the same way as previously: create a workspace, clone the repositories, source ROS2 (foxy), build using colcon, start the example.

1. Start the PX4 simulator and agent code in the same way as before (you can use the existing terminals if they are still running).
1. Create and navigate to a new workspace:

   ```sh
   mkdir -p ~/ws_sensor_combined/src/
   cd ~/ws_sensor_combined/src/
   ```
1. Clone the example repository and px4_msgs:

   ```sh
   git clone https://github.com/PX4/px4_msgs.git
   git clone https://github.com/PX4/px4_ros_com.git
   ```
1. Source ROS2 for the current terminal and build using colcon
   
   ```sh
   cd ..
   source /opt/ros/foxy/setup.bash
   colcon build
   ```

1. Now source the workspace and launch the example.
   Note here that we use `ros2 launch`, which is described below.

   ```
   source install/local_setup.bash
   ros2 launch px4_ros_com sensor_combined_listener.launch.py
   ```

If this is working yhou should see data being printed on the terminal/console where you launched the ROS listener:

```sh
RECEIVED DATA FROM SENSOR COMBINED
================================
ts: 870938190
gyro_rad[0]: 0.00341645
gyro_rad[1]: 0.00626475
gyro_rad[2]: -0.000515705
gyro_integral_dt: 4739
accelerometer_timestamp_relative: 0
accelerometer_m_s2[0]: -0.273381
accelerometer_m_s2[1]: 0.0949186
accelerometer_m_s2[2]: -9.76044
accelerometer_integral_dt: 4739
```

You can also verify the rate of the message using `ros2 topic hz`.
E.g. in the case of `sensor_combined` use `ros2 topic hz /fmu/out/sensor_combined`:
```sh
average rate: 248.187
  min: 0.000s max: 0.012s std dev: 0.00147s window: 2724
average rate: 248.006
  min: 0.000s max: 0.012s std dev: 0.00147s window: 2972
average rate: 247.330
  min: 0.000s max: 0.012s std dev: 0.00148s window: 3212
average rate: 247.497
  min: 0.000s max: 0.012s std dev: 0.00149s window: 3464
average rate: 247.458
  min: 0.000s max: 0.012s std dev: 0.00149s window: 3712
average rate: 247.485
  min: 0.000s max: 0.012s std dev: 0.00148s window: 3960
```

#### Listener source code

<!-- note, code changed below, and we also need to talk about QOS -->

Lets take as an example the `sensor_combined_listener.cpp` node under `px4_ros_com/src/examples/listeners`.

The code first imports the C++ libraries needed to interface with the ROS 2 middleware and the required message header file:

```cpp
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
```

Then it creates a `SensorCombinedListener` class that subclasses the generic `rclcpp::Node` base class.

```cpp
/**
 * @brief Sensor Combined uORB topic data callback
 */
class SensorCombinedListener : public rclcpp::Node
{
```

This creates a callback function for when the `sensor_combined` uORB messages are received (now as RTPS/DDS messages), and outputs the content of the message fields each time the message is received.

```cpp
public:
	explicit SensorCombinedListener() : Node("sensor_combined_listener") {
		subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
			"fmu/sensor_combined/out",
			10,
			[this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED SENSOR COMBINED DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "          << msg->timestamp    << std::endl;
			std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
			std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
			std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
			std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
			std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
			std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
			std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
			std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
			std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;
		});
	}
```

The lines below create a publisher to the `sensor_combined` uORB topic, which can be matched with one or more compatible ROS2 subscribers to the `fmu/sensor_combined/out` ROS2 topic.

```cpp
private:
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;
};
```

The instantiation of the `SensorCombinedListener` class as a ROS node is done on the `main` function.

```cpp
int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_combined listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorCombinedListener>());

	rclcpp::shutdown();
	return 0;
}
```


### ROS 2 Advertiser

A ROS 2 advertiser node publishes data into the DDS/RTPS network (and hence to the PX4 Autopilot).

Taking as an example the `debug_vect_advertiser.cpp` under `px4_ros_com/src/advertisers`, first we import required headers, including the `debug_vect` msg header.

```cpp
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/debug_vect.hpp>

using namespace std::chrono_literals;
```

Then the code creates a `DebugVectAdvertiser` class that subclasses the generic `rclcpp::Node` base class.

```cpp
class DebugVectAdvertiser : public rclcpp::Node
{
```

The code below creates a function for when messages are to be sent.
The messages are sent based on a timed callback, which sends two messages per second based on a timer.

```cpp
public:
	DebugVectAdvertiser() : Node("debug_vect_advertiser") {
		publisher_ = this->create_publisher<px4_msgs::msg::DebugVect>("fmu/debug_vect/in", 10);
		auto timer_callback =
		[this]()->void {
			auto debug_vect = px4_msgs::msg::DebugVect();
			debug_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
			std::string name = "test";
			std::copy(name.begin(), name.end(), debug_vect.name.begin());
			debug_vect.x = 1.0;
			debug_vect.y = 2.0;
			debug_vect.z = 3.0;
			RCLCPP_INFO(this->get_logger(), "\033[97m Publishing debug_vect: time: %llu x: %f y: %f z: %f \033[0m",
                                debug_vect.timestamp, debug_vect.x, debug_vect.y, debug_vect.z);
			this->publisher_->publish(debug_vect);
		};
		timer_ = this->create_wall_timer(500ms, timer_callback);
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<px4_msgs::msg::DebugVect>::SharedPtr publisher_;
};
```

The instantiation of the `DebugVectAdvertiser` class as a ROS node is done on the `main` function.

```cpp
int main(int argc, char *argv[])
{
	std::cout << "Starting debug_vect advertiser node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DebugVectAdvertiser>());

	rclcpp::shutdown();
	return 0;
}
```


### Offboard Control

For a complete reference example on how to use Offboard control with PX4, see: [ROS 2 Offboard control example](../ros/ros2_offboard_control.md).

<!--

## Manual Workspace Setup with a ROS1 compatible workspace (FYI Only)

:::note
This is provided to help you better understand the build process and how to include the ROS1 workspace.
It is not needed to build or use ROS 2.
It additionally includes instructions for building the `ros1_bridge` package, which is used in [ROS (1) via ROS 2 Bridge](../ros/ros1_via_ros2.md).
:::

This section describes the process to *manually* setup your workspace and build the `px4_ros_com`, `px4_msgs`, and `ros1_bridge` package.
The topic effectively explains the operation of the `build_ros2_workspace.bash` script in the [installation instructions](#build-ros-2-workspace)).


**To build the ROS 2 workspace only:**

1. `cd` into `px4_ros_com_ros2` dir and source the ROS 2 environment.
   Don't mind if it tells you that a previous workspace was set before:

   ```sh
   cd ~/px4_ros_com_ros2
   source /opt/ros/foxy/setup.bash
   ```

2. Build the workspace:

   ```sh
   colcon build --symlink-install --event-handlers console_direct+
   ```        

To build both ROS 2 and ROS (1) workspaces (replacing the previous steps):

1. `cd` into `px4_ros_com_ros2` dir and source the ROS 2 environment.
   Don't mind if it tells you that a previous workspace was set before:

   ```sh
   source /opt/ros/foxy/setup.bash
   ```

1. Clone the `ros1_bridge` package so it can be built on the ROS 2 workspace:

   ```sh
   git clone https://github.com/ros2/ros1_bridge.git -b dashing ~/px4_ros_com_ros2/src/ros1_bridge
   ```

1. Build the `px4_ros_com` and `px4_msgs` packages, excluding the `ros1_bridge` package:

   ```sh
   colcon build --symlink-install --packages-skip ros1_bridge --event-handlers console_direct+
   ```

   :::note
   `--event-handlers console_direct+` only serve the purpose of adding verbosity to the `colcon` build process, and can be removed if one wants a more "quiet" build.
   :::

1. Then build the ROS(1) packages side.
   First open a **new** terminal window and source the ROS(1) environment that was installed on the system:

   ```sh
   source /opt/ros/melodic/setup.bash
   ```

1. Build the `px4_ros_com` and `px4_msgs` packages on the ROS end (using the terminal opened in the previous step):

   ```sh
   cd ~/px4_ros_com_ros1 && colcon build --symlink-install --event-handlers console_direct+
   ```

1. Open another new terminal and then source the environments and workspaces in the order listed below:

   ```sh
   source ~/px4_ros_com_ros1/install/setup.bash
   source ~/px4_ros_com_ros2/install/setup.bash
   ```

1. Finally, build the `ros1_bridge`:

   ```sh
   cd ~/px4_ros_com_ros2 && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure --event-handlers console_direct+
   ```

   :::note
   The build process may consume a lot of memory resources.
   On a resource limited machine, reduce the number of jobs being processed in parallel (e.g. set environment variable `MAKEFLAGS=-j1`).
   For more details on the build process, see the build instructions on the [ros1_bridge](https://github.com/ros2/ros1_bridge) package page.
   :::
-->

<!-- 
### Cleaning the workspaces

After building the workspaces there are many files that must be deleted before you can do a clean/fresh build (for example, after you have changed some code and want to rebuild).

Unfortunately *colcon* does not currently have a way of cleaning the generated **build**, **install** and **log** directories, so these directories must be deleted manually.

The **clean_all.bash** script (in **px4_ros_com/scripts**) is provided to ease this cleaning process, this script can be used to clean all of the workspace options listed above (ROS 2, ROS 1, and Both)

The most common way of using it is by passing it the ROS (1) workspace directory path (since it's usually not on the default path):

```sh
source clean_all.bash --ros1_ws_dir <path/to/px4_ros_com_ros1/ws>
```

:::tip
Like the build scripts, the `clean_all.bash` script also has a `--help` guide.
:::
-->


## Troubleshooting

### Missing dependencies

The standard installation should include all the tools needed by ROS2.

If any are missing, they can be added separately:
- **`colcon`** build tools should be in the development tools.
  It can be installed using:
  ```sh
  sudo apt install python3-colcon-common-extensions
  ```
- The Eigen3 library used by the transforms library should be in the both the desktop and base packages.
  It can be installed using:
  ```sh
  sudo apt install ros-foxy-eigen3-cmake-module
  ```


## Additional information

- [ROS2 in PX4: ROS2 in PX4: Technical Details of a Seamless Transition to XRCE-DDS](https://www.youtube.com/watch?v=F5oelooT67E) - Pablo Garrido & Nuno Marques (youtube)
- [DDS and ROS middleware implementations](https://github.com/ros2/ros2/wiki/DDS-and-ROS-middleware-implementations)




<!--

<!--
1. To start the agent
Source the compiled workspace and run the Agent (assuming a the default sitl scenario where the client tries to connect using udp4 on port 8888 like here https://gist.github.com/julianoes/adbf76408663829cd9aed8d14c88fa29#px4-sitl-with-gazebo-classic)
source install/local_setup.bash
MicroXRCEAgent udp4 -p 8888

Separate terminal
- Start PX4: make px4_sitl gazebo-classic

Start seeing items being created.

Separate terminal
- Can use ROS2 command to query the topics
source /opt/ros/foxy/setup.bash
ros2 topic list

- But if want to look at detail of topic need to do so with a local build that has px4_msgs built. - ie. locally sourced

source install/local_setup.bash
ros2 topic echo /fmu/out/vehicle_status

Now lets try jae examples. https://gist.github.com/julianoes/adbf76408663829cd9aed8d14c88fa29#offboard-example-and-px4_msgs
BUT wthout ros.com

mkdir example_ws
cd example_ws
mkdir src
git clone https://github.com/PX4/px4_msgs.git
//// git clone https://github.com/PX4/px4_ros_com.git src/px4_ros_com
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git src/px4-offboard
Then build it:
colcon build

source install/local_setup.bash
Test: ros2 topic echo /fmu/out/vehicle_status

ros2 launch px4_offboard offboard_position_control.launch.py

WORKS IF You have installed the desktop version.
SO, need to find out how to modify the listener/sender thingy to work, and not depend on agent starting.


Fix up the roscom?
ros2 launch px4_ros_com sensor_combined_listener.launch.py

-----

create a ros2 workspace
mkdir -p ~/px4_ros_ws/src
cd ~/px4_ros_ws/src
clone the dependencies px4_msgs and micro XRCE-DDS Agent
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
Source the main environment and compile the workspace
cd ~/px4_ros_ws/
source /opt/ros/humble/setup.bash
colcon build
Source the compiled workspace and run the Agent (assuming a the default sitl scenario where the client tries to connect using udp4 on port 8888 like here https://gist.github.com/julianoes/adbf76408663829cd9aed8d14c88fa29#px4-sitl-with-gazebo-classic)
source install/local_setup.bash
MicroXRCEAgent udp4 -p 8888

-->