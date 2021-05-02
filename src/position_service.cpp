
#include <inttypes.h>
#include <memory>
#include "rt2_assignment1/srv/random_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macrohpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{

	class PositionServer : public rclcpp::Node
	{
		public:
			PositionServer(const rclcpp::NodeOptions & options) : Node("random_position_server",options)
			{
				 rdm_service = this->create_service<RandomPosition>(
					"/position_server", std::bind(&PositionServer::myrandom, this, _1, _2, _3));
			}
			
		private:
			
			double randMToN(double M, double N)
				{   return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


			bool myrandom (const std::shared_ptr<rmw_request_id_t> request_header,
							rt2_assignment1::RandomPosition::Request &req, 
							rt2_assignment1::RandomPosition::Response &res)
			{
				(void)request_header; // to avoid warnings
				res->x = randMToN(req->x_min, req->x_max);
				res->y = randMToN(req->y_min, req->y_max);
				res->theta = randMToN(-3.14, 3.14);
				return true;
			}
			
			rclcpp::Service<RandomPosition>::SharedPtr rdm_service;
	};
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1:PositionServer)
