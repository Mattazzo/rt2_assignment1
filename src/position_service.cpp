/**
 * \file position_service.cpp
 * \brief This file implement a server to get a random position for the robot
 * \author Matteo Azzini
 * \version 1.0
 * \date 23/07/2021
 * 
 * \details
 * 	
 * Services:<BR>
 *	° /position_server	 
 * 
 * 
 * Description:
 * 
 * Position server that return a random position between the bounds specified
 * in the request
 */

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 * \brief Function to get a random number from the specified interval
 * 
 * \param M: lower bound 
 * \param N: upper bound
 * 
 * \return a random number between M and N
 * 
 * Function to get a random number from the specified interval
 * 
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/**
 * \brief Function to get a random number from the specified interval
 * 
 * \param req: bounds for the interval to choose the random position  
 * \param res: random position 
 * 
 * \return always true
 * 
 * Service function to get a random position between the bounds specified
 * in the request
 * 
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


/**
 * \brief Function to get a random number from the specified interval
 * 
 * \param argc: number of argument 
 * \param argv: pointer to argument vector
 * 
 * \return always zero
 * 
 * Main function, declares a server for position_server to get a random 
 * position
 */
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
