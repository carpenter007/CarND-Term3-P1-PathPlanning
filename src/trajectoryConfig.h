/*
 * costWeights.h
 *
 *  Created on: Jul 11, 2018
 *      Author: elias
 */

#ifndef TRAJECTORYCONFIG_
#define TRAJECTORYCONFIG_

const double WEIGHT_COSTS_NEAR_APROACH = 10.0;
const double WEIGHT_COSTS_FOR_TRAJECTORY_CHANGE = 2.0;
const double REDUCE_VELOCITY_COST_THRESHOLD = 1.0;
const double WEIGHT_COSTS_IS_LANE_CURRENTLY_SAFE = 10.0;
const double WEIGHT_COSTS_DISTANCE_TO_CARS = 1.0;
const double WEIGHT_COSTS_SLOWER_VELOCITY = 4.0;

const int NUM_LANES = 3;

const int FAR_RANGE_IN_METERS = 150;
const int SHORT_RANGE_IN_METERS = 15;
const int MID_RANGE_IN_METERS = 75;
const double MAX_SPEED_MPH = 49.5;


#endif /* TRAJECTORYCONFIG_ */
