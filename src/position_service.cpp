/**
 * Position server that return a random position between the bounds specified
 * in the request
 */

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 * Function to get a random number from the specified interval
 * 
 * @param
 * 		- M: lower bound 
 * 		- N: upper bound
 * @return
 * 		- a random number between M and N
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
 * Service function to get a random position between the bounds specified
 * in the request
 * 
 * @param
 * 		- req: bounds for the interval to choose the random position 
 * 		- res: random position 
 * @return
 * 		- always true
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
 *  Main function, declares a server for position_server to get a random 
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
