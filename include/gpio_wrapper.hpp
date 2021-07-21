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
 *	 	 above copyright notice, this list of conditions and the following disclaimer.
 *     * Neither the name of the CAVAS, UB SUNY, nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 */

/*
 * Author & Maintainer: Foad Hajiaghajani
 */

#ifndef _cptl_gpio_wrapper /* Prevent multiple inclusion */
#define _cptl_gpio_wrapper

#include <time.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <bcm2835.h>
#include <bits/stdc++.h>
#include <vector>
#include <algorithm>
#include <signal.h>

#define OFF 0
#define ON 1
std::vector<int> pins;

class GPIO
{
public:
	
	GPIO(){
	}
	bool init(int pin_)
	{
		if (!bcm2835_init())
		{
			printf("Failed to setup BCM2835 GPIO interface!\n");
			return false;
		}
		
		
		this->pin = pin_;
		if (verify_pin())
		{
			pins.push_back(this->pin);
			bcm2835_gpio_fsel(this->pin, BCM2835_GPIO_FSEL_OUTP);
			lightOff();
			return true;
		}
		else
		{
			err_gpio_used(this->pin);
			return false;
			
		}
	}
	
	void lightOff()
	{
		bcm2835_gpio_write(this->pin, LOW);
	}	
	void lightOn() 
	{
		bcm2835_gpio_write(this->pin, HIGH);
	}
	static void allLightsOff() 
	{
		for (int i=0; i<pins.size()-1; i++)
			bcm2835_gpio_write(pins.at(i), LOW);
		bcm2835_gpio_write(pins.at(pins.size()-1), HIGH);
	}
	static void interruptHandler(const int signal)
	{
		close();
		allLightsOff();
		exit(0);
	}
	static void delay(int t_)
	{
		bcm2835_delay(t_);
	}
	static void close()
	{
		bcm2835_close();
	}
private:
	int pin;
	bool status;
	
	bool verify_pin()
	{
		std::vector<int>::iterator it;
		it = find(pins.begin(),pins.end(),this->pin);
		if (it != pins.end())
			return false;
		else
			return true;
	}
	void err_gpio_used (int pin_)
	{
		printf("ERROR GPIO pin %d already used. \n", pin_); 
	}
};
#endif /* ndef _cptl_gpio_wrapper */
