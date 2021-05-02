#include <inttypes.h>
#include <memory>
#include "rt2_assignment1/srv/command.h"
#include "rt2_assignment1/srv/position.h"
#include "rt2_assignment1/srv/random_position.h"
#include "rclcpp_components/register_node_macrohpp"


using RandomPosition = rt2_assignment1::srv::RandomPosition;
using Command = rt2_assignment1::srv::Command;
using Position = rt2_assignment1::srv::Position;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{
	
	class StateMachine : public rclcpp::Node
	{
		public:
			bool start = false;
			
			StateMachine(const rclcpp:NodeOptions & options) : Node("state_machine",options)
			{
				//command service server
				cmd_service = this->create_service<Command>(
					"/user_interface", std::bind(&StateMachine::double user_interface, this, _1, _2, _3));
				
				//random position service client
				rdm_pos_client = this->create_client<RandomPosition>("/postion_server")
				while (!rdm_pos_client -> wait_for_service(std::chrono::econds(1))){
					if(rclcpp::ok()){
						RCLCPP_ERROR(this->get_logger(), "random position client interrupted");
						return;
					}
				}
							
				//go to point service client
				goto_client = this->create_client<Position>("/go_to_point")
				while (!goto_client -> wait_for_service(std::chrono::econds(1))){
					if(rclcpp::ok()){
						RCLCPP_ERROR(this->get_logger(), "go to point client interrupted");
						return;
					}
				}	
				
				this->fsm()
				
			}
			
			void fsm(){
				
				auto rp_request = std::make_shared<RandomPosition::Request>();
				rp_request->x_max = 5.0;
				rp_request->x_min = -5.0;
				rp_request->y_max = 5.0;
				rp_request->y_min = -5.0;
				
				auto pos = std::make_shared<Position::Request>();
				
				while(ros::ok()){
					ros::spinOnce();
					
					if (start){
						auto rp_response = rdm_pos_client->async_sendrequest(rp_request);

						pos->x = rp_response->x;
						pos->y = rp_response->y;
						pos->theta = rp_response->theta;
						
						std::cout << "\nGoing to the position: x= " << rp_request->x << " y= " << rp_request->y << " theta = " <<rp_request->theta << std::endl;
						
						auto pos_response = goto_client->async_sendrequest(pos);
						std::cout << "Position reached" << std::endl;
					}
				}
			}		
			
		private:
			bool user_interface(const std::shared_ptr<rmw_request_id_t> request_header,
								rt2_assignment1::Command::Request &req, 
								rt2_assignment1::Command::Response &res)
			{
				(void)request_header; // to avoid warnings
				
				if (req->command == "start"){
					start = true;
				}
				else {
					start = false;
				}
				return true;
			}
			
			rclcpp::Service<Command>::SharedPtr cmd_service;
	};		
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1:StateMachine)






