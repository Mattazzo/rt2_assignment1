/**
 * Class PositionServer in namespace rt2_assignment1, it implements a server
 * node for the service RandomPosition
 */
#include <inttypes.h>
#include <memory>
#include "rt2_assignment1/srv/random_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{
	
	/**
	 * Class PositionServer which implements the server of RandomPosition
	 * service, it receives a request with bounds of the interval within 
	 * choose x and y coordinates, it sends back as response the random 
	 * position (x,y coordinates and the heading theta)  
	 * 
	 */
	class PositionServer : public rclcpp::Node
	{
		public:
			/**
			 * Class constructor, it contains declaration of a server for 
			 * RandomPosition service
			 * 
			 * @param 
			 * 		options: NodeOptions object to create the node
			 */
			PositionServer(const rclcpp::NodeOptions & options) : Node("random_position_server",options)
			{
				 rdm_service = this->create_service<RandomPosition>(
					"/position_server", std::bind(&PositionServer::myrandom, this, _1, _2, _3));
			}
			
		private:
			/**
			 * Method to return a random number in a certain interval
			 * 
			 * @param
			 * 		M: lower bound 
			 * 		N: upper bound
			 * 
			 * @return
			 * 		random number between M and N 
			 */
			double randMToN(double M, double N)
				{   return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

			
			/**
			 * Function of server for RandomPosition service. It receives 
			 * as request the interval bound within choose the x and y coordinates.
			 * It gives back as response the random position to be reached by the robot 
			 * 
			 * @param
			 * 		request_header: parameter to avoid warnings
			 * 		req: request for the RandomPosition service, interval bounds for x and y coordinates
			 * 		res: random goal position (x, y coordinates and heading theta) 
			 * 
			 * @return
			 * 		True
			 */
			bool myrandom (const std::shared_ptr<rmw_request_id_t> request_header,
					const std::shared_ptr<RandomPosition::Request> req, 
					const std::shared_ptr<RandomPosition::Response> res)
			{
				(void)request_header; // to avoid warnings
				res->x = randMToN(req->x_min, req->x_max);
				res->y = randMToN(req->y_min, req->y_max);
				res->theta = randMToN(-3.14, 3.14);
				return true;
			}
			
			rclcpp::Service<RandomPosition>::SharedPtr rdm_service;		//server for RandomPosition service
	};
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::PositionServer)


