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

#include "cptl_ros_driver.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cptl_node");
  cptl_ros::CPTLNode node;
  ros::spinOnce();
  node.run();
  
  return 0;
}
