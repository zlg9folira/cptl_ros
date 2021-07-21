/*
 * Copyright (c) 2021, Foad Hajiaghajani
 * Connected Autonomous Vehicles Research and Systems (CAVAS)
 * University at Buffalo, State University of New York
 * All rights reserved.
 */

/* Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code AND/OR binary outputs must retain the
 *	 	   above copyright notice, this list of conditions and the following disclaimer.
 *     * Neither the name of the CAVAS, UB SUNY, nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 */

/*
 * Author & Maintainer: Foad Hajiaghajani
 */
 
 
#ifndef _cptl_ros_driver /* Prevent multiple inclusion */
#define _cptl_ros_driver

#include <string>
#include <ros/ros.h>
#include <bcm2835.h>
#include <thread>
#include <chrono>
#include <ctime>
#include <cmath>
#include "gpio_wrapper.hpp"
#include <cavas_msgs/SPaT.h>
#include <std_msgs/Float64.h>

namespace cptl_ros
{
class CPTLNode
{
public:
  CPTLNode();
  CPTLNode(ros::NodeHandle nh);
  /**
   * @brief Start's the nodes threads to start the CPTL.
  */
  void run();
  /**
  * @brief Verify ROS status.
  */
  bool good_to_go();
  /**
  * @brief In case of phase timeout, return next phase in order.
  */
  int get_next_phase(int i);
  /**
  * @brief Construct the SPaT message and publish
  */
  void publish_spat (int it_, float tts);
  /**
  * @brief starts timer for signal phase control
  */
  void phase_timer_start();
  /**
  * @brief starts timer for signal phase control
  */
  void phase_timer_stop();
  /**
  * @brief returns elapsed time in milliseconds
  */
  double getElapsedMilliseconds();
  /**
  * @brief returns elapsed time in seconds
  */
  double getElapsedSeconds();

private:
  /**
  * @brief Initialize the node and read config parameters.
  */
  bool init();
  
  ros::NodeHandle nh_;
  ros::Rate loop_rate;
  int infra_id;
  bool infinit;
  std::string frame_id;
  std::string topic_id;
  std::vector<float> position;
  std::vector<std::string> phase_order;
  std::vector<int> pin_order;
  std::vector<int> timing_order; //sec
  std::string infra_class;
  int phase_num;
  std::string curr_phase;
  double time_to_switch;
  double time_start;
  double time_end;
  ros::Publisher spat_pub_;
  std::chrono::time_point<std::chrono::system_clock> m_StartTime;
  std::chrono::time_point<std::chrono::system_clock> m_EndTime;
  bool m_bRunning;
};
} // namespace
#endif  // _cptl_ros_driver
 
 
