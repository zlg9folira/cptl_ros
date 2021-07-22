cptl_ros
======
ROS driver to manage and publish **Signal Phase and Timing (SPaT)** information on single Connected Portable Traffic Light (CPTL) system in [CAVAS ecosystem](https://github.com/zlg9folira/CAVAS_ecosystem).

The `cptl_ros` package has been tested under ROS Kinethic and Melodic on Ubuntu 16.04, Ubuntu 18.04 and Raspberry Pi 64bit OS. The driver is compatible with Jetson Nano and Raspberry Pi 4B hardware.

**Latest releases**: 

| Driver version | CAVAS_ecosystem supported | CPTL hw supported |
| :---:         | :---:     | :---: |
| [v1.1](https://github.com/zlg9folira/cptl_ros/releases/tag/1.1)   | [v1.0](https://github.com/zlg9folira/CAVAS_ecosystem/releases/tag/v1.0) or newer   | V2.1 or newer   |
| [v1.0](https://github.com/zlg9folira/cptl_ros/releases/tag/1.0)     | [v1.0](https://github.com/zlg9folira/CAVAS_ecosystem/releases/tag/v1.0) or newer       | V2.1 or newer  |

**WARNING:** GPIO output pins are device/protocol specific and must be verified or modified according to the hardware in-use. This driver requires `BCM2835`-compatible processor to run properly.

## Dependencies

 - Broadcom `BCM2835` v1.60 library ([link](https://www.airspayce.com/mikem/bcm2835/))
 - ROS Kinetic ([link](http://wiki.ros.org/kinetic/Installation)) or Melodic ([link](http://wiki.ros.org/melodic/Installation/Ubuntu))
 - cavas_msgs ([link](https://github.com/zlg9folira/CAVAS_ecosystem))
 - std_msgs ([link](http://wiki.ros.org/std_msgs))

## Installation and Configuration

Install `BCM2835` v1.60 C library or visit https://www.airspayce.com/mikem/bcm2835/ for latest version:
```
mkdir ~/BCM2835 && cd ~/BCM2835
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.60.tar.gz
tar zxvf bcm2835-1.60.tar.gz 
cd bcm2835-1.60/
sudo ./configure
sudo make && sudo make check && sudo make install
```

The infrastructure will publish SPaT information in the form of `cavas_msgs::SPaT`. The package `cavas_msgs` contains all the messages required accross CAVAS ecosystem. To install `cavas_msgs`, simply clone and install `cavas_ecosystem` core package:
```
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws/src
git clone https://github.com/zlg9folira/CAVAS_ecosystem.git && cd ..
catkin_make --only-pkg-with-deps cavas_ecosystem
```

Finally, download and install `cptl_ros` driver:
```
cd ~/catkin_ws/src
git clone https://github.com/zlg9folira/cptl_ros.git && cd ..
catkin_make --only-pkg-with-deps cptl_ros
```

## Usage

To launch the driver, simply run the following command on the CPTL infrastructure:
```
roslaunch cptl_ros cptl_3phase.launch
```

### User-defined parameters

 - `id_`: The unique ID of the infrastructure (default 0)
 - `total_phase_number_`: Number of signal phases supported by traffic light (default 3)
 - `infra_class_`: Target signal class of the infrastructure, e.g., `pedestrian', 'vehicle` (default `vehicle`)
 - `duration_red_`: Timing in seconds for red signal (default `10`) 
 - `duration_amber_`: Timing in seconds for amber (yellow) signal (default `4`) 
 - `duration_green_`: Timing in seconds for green signal (default `10`) 
 - `gpio_out_red_`: Pin number connected to red signal relay switch (default `16`)
 - `gpio_out_amber_`: Pin number connected to amber (yellow) signal relay switch (default `14`)
 - `gpio_out_green_`: Pin number connected to green signal relay switch (default `12`)
 - `infra_latitude_`: Latitude coordinate of the infrastructure (default `0.0`)
 - `infra_longitude_`: Longitude coordinate of the infrastructure (default `0.0`)
 - `run_for_ever_`: Flag variable to set for infinit cycle count for SPaT (default `true`)
 - `topic_prefix_`: Naming convention for SPaT ROS topic prefix (default `/infra/cptl_`)
 - `frame_`: Naming convention for SPaT ROS topic frame ID (default `cptl`)

**Note:** ROS topic naming convention follows the given infrastructure ID. It means that if `id_` is set to `0`, then SPaT messages will be published on `/infra/cptl_0` topic. 


## Advertised Topic(s)

 - `/infra/cptl_0` ([cavas_msgs/SPaT](https://github.com/zlg9folira/CAVAS_ecosystem/tree/main/cavas_msgs))
   containing global position, current phase, next phase, time to next phase, etc., for particular CPTL system.

## Meta
### Credit:
Foad Hajiaghajani - UB CAVAS


