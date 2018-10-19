/*
Adept MobileRobots Robotics Interface for Applications (ARIA)
Copyright (C) 2004-2005 ActivMedia Robotics LLC
Copyright (C) 2006-2010 MobileRobots Inc.
Copyright (C) 2011-2015 Adept Technology, Inc.
Copyright (C) 2016 Omron Adept Technologies, Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
Adept MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
Adept MobileRobots, 10 Columbia Drive, Amherst, NH 03031; +1-603-881-7960
*/
#include "Aria.h"
#include <iostream>

/** @example gotoActionExample.cpp Uses ArActionGoto to drive the robot in a square
 
  This program will make the robot drive in a 2.5x2.5 meter square by
  setting each corner in turn as the goal for an ArActionGoto action.
  It also uses speed limiting actions to avoid collisions. After some 
  time, it cancels the goal (and the robot stops due to a stopping action) 
  and exits.

  Press escape to shut down Aria and exit.

  Modifications: 
  Trying to convert points from the scene coordinates to robot 
  coordinates.
  Adding rotation in the last part.
*/

double* changeCoordinates (double xE, double yE, double thE)
{
	double xR, yR, thR;
	xR = yE - 450.0;
	yR = -xE + 450.0;
	thR = thE - 90.0;
	double coordinatesR[3] = {xR,yR,thR};
	
	return coordinatesR;
}

double getX (double yE)
{
	double xR = yE - 450.0;
	return xR;
}

double getY (double xE)
{
	double yR = -xE + 450.0;
	return yR;
}

double getTh (double thE)
{
	double thR = thE - 90.0;
	return thR;
}
int main(int argc, char **argv)
{
  int xCoordinates[8][17];
  int yCoordinates[8][17];

  for (int i=0; i < 8; i++)
	  {
		  for (int j=0; j < 17; j++)



  
  std::cout<<coordinates[0][1]<<std::endl;
  //return 0;
}
