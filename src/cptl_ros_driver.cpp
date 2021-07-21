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

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <chrono>
#include <ctime>
#include <cmath>
#include <thread>
#include <wiringPi.h>
#include "cptl_ros_driver.h"
#include <std_msgs/Float64.h>

#define txtblack    "\x1b[30m"
#define txtred      "\x1b[31m"
#define txtgreen    "\x1b[32m"
#define txtyellow   "\x1b[33m"
#define txtblue     "\x1b[34m"
#define txtmagenta  "\x1b[35m"
#define txtcyan     "\x1b[36m"
#define txtwhite    "\x1b[37m"
#define txtcolend   "\x1b[0m"

int warm_up_time = 3000; // in ms

namespace cptl_ros
{
CPTLNode::CPTLNode(ros::NodeHandle nh) :
  nh_(nh),
  loop_rate(10),
  position(2),
  m_bRunning(false)
{
  init();
}
CPTLNode::CPTLNode():
  nh_(),
  loop_rate(10),
  position(2),
  m_bRunning(false)
{
  init();
}

void CPTLNode::phase_timer_start()
{
    m_StartTime = std::chrono::system_clock::now();
    m_bRunning = true;
}
  
void CPTLNode::phase_timer_stop()
{
    m_EndTime = std::chrono::system_clock::now();
    m_bRunning = false;
}
  
double CPTLNode::getElapsedMilliseconds()
{
    std::chrono::time_point<std::chrono::system_clock> endTime;
    
    if(m_bRunning)
      endTime = std::chrono::system_clock::now();
    else
      endTime = m_EndTime;
    return std::chrono::duration_cast<std::chrono::milliseconds>(endTime - m_StartTime).count();
}
  
double CPTLNode::getElapsedSeconds()
{
  return getElapsedMilliseconds() / 1000.0;
}
    
bool CPTLNode::init()
{
  int gpio_r_,gpio_g_,gpio_a_;
  int duration_red_,duration_green_,duration_amber_;
  double lat_, lon_;
  
  // Read parameters from launch file
  nh_.getParam("/cptl_ros/infra_id", infra_id);
  nh_.getParam("/cptl_ros/phase_num", phase_num);
  nh_.getParam("/cptl_ros/infra_class",infra_class); //vehicle or /pedestrian
  nh_.getParam("/cptl_ros/topic_id", topic_id);
  nh_.getParam("/cptl_ros/frame_id", frame_id);
  nh_.getParam("/cptl_ros/latitude", lat_);
  nh_.getParam("/cptl_ros/longitude", lon_);
  nh_.getParam("/cptl_ros/infinit", infinit);
  nh_.getParam("/cptl_ros/gpio_r", gpio_r_);
  nh_.getParam("/cptl_ros/gpio_a", gpio_a_);
  nh_.getParam("/cptl_ros/gpio_g", gpio_g_);
  nh_.getParam("/cptl_ros/duration_red", duration_red_);
  nh_.getParam("/cptl_ros/duration_amber", duration_amber_);
  nh_.getParam("/cptl_ros/duration_green", duration_green_);
  if (gpio_r_ <= 0 || gpio_a_ <= 0 || gpio_g_ <= 0)
  {
      printf("%sError:%s\tunsupported GPIO pin number. Check inputs...%s!",txtred,txtcolend);
      return false;
  }
  position.at(0) = lat_; //.push_back(lat_);
  position.at(1) = lon_; //.push_back(lon_);

  switch (phase_num)
  {
    case 2:
      phase_order.push_back("GREEN");
      pin_order.push_back(gpio_g_);
      timing_order.push_back(duration_green_);
      phase_order.push_back("RED");
      pin_order.push_back(gpio_r_);
      timing_order.push_back(duration_red_);
      break;
    case 3:
      phase_order.push_back("GREEN");
      pin_order.push_back(gpio_g_);
      timing_order.push_back(duration_green_);
      phase_order.push_back("AMBER");
      pin_order.push_back(gpio_a_);
      timing_order.push_back(duration_amber_);
      phase_order.push_back("RED");
      pin_order.push_back(gpio_r_);
      timing_order.push_back(duration_red_);
      break;
    default:
      printf("%sError:%s\tInvalid phase number!\n\tCurrently supporting phase numbers: {2-3}\n",txtred,txtcolend);
      return false;
  }
  spat_pub_ = nh_.advertise<cavas_msgs::SPaT>(topic_id, 1);

  return true;
}

int CPTLNode::get_next_phase(int i)
{
  int N = phase_num;
  if (i != N-1)
    return i+1; // next phase in order
  else
    return 0;   // reset phase
}

bool CPTLNode::good_to_go()
{
  return (nh_.ok());
}

void CPTLNode::publish_spat(int it_, float tts)
{
    // construct SPaT message
    cavas_msgs::SPaT spat;
    spat.infra_id	= infra_id;		
    spat.infra_class = infra_class;	
    spat.total_phase = phase_num;
    spat.latitude = position.at(0);	
    spat.longitude = position.at(1);		
    spat.phase = phase_order.at(it_);
    spat.next_phase = phase_order.at(get_next_phase(it_));
    spat.time_to_change = tts;
    spat.header.stamp = ros::Time::now();
    spat.header.frame_id = frame_id;
    
    // publish SPaT message
    spat_pub_.publish(spat);
}

void CPTLNode::run()
{  
  // Initialize CPTL signals
  GPIO RED; GPIO AMBER, GREEN;   
  signal (SIGINT, GPIO::interruptHandler);
  if (RED.init(pin_order.at(2)) && AMBER.init(pin_order.at(1)) && GREEN.init(pin_order.at(0)))
    printf("%sSuccess:%s\tROS node and GPIO outputs initialized %s(Warming up...)\n%s",txtgreen,txtcolend,txtyellow,txtcolend);
  else
    printf("%sError:%s\tROS and GPIO nodes failed to initialize.\n",txtred,txtcolend);
  
  // wait for other services (if any)
  delay(warm_up_time); 
    
  bool phase_timeout = true;
  int phase_itr = -1; //first itr
  //Run CPTL schedule Green --> Amber --> Red
  printf("%sNode stable:%s\tPublishing SPaT messages...\n",txtgreen,txtcolend,warm_up_time/1000);
  while(good_to_go())
  {
    if (phase_timeout)
    {
      phase_timeout = false;
      phase_itr = get_next_phase(phase_itr);
      phase_timer_start();
    }

    float time_to_switch = timing_order.at(phase_itr) - getElapsedSeconds();
    // construct the message and publish
    publish_spat(phase_itr, time_to_switch);
    
    // check phase timeout
    if (time_to_switch < 0.01f)
    {
      phase_timer_stop();
      phase_timeout = true;
    }

  loop_rate.sleep();
  } // while(CPTLNode::good_to_go())
} // run()
} //namespace
