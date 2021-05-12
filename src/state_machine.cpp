
#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/random_position.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition;
using Command = rt2_assignment1::srv::Command;
using Position = rt2_assignment1::srv::Position;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

float x,y,theta;
rclcpp::executors::SingleThreadedExecutor exec;

namespace rt2_assignment1{
	
	class StateMachine : public rclcpp::Node
	{
		public:
			
			StateMachine(const rclcpp::NodeOptions & options) : Node("state_machine",options)
			{
				//command service server
				cmd_service = this->create_service<Command>(
					"/user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));
				
				//random position service client
				rdm_pos_client = this->create_client<RandomPosition>("/position_server");
				while (!rdm_pos_client -> wait_for_service(std::chrono::seconds(1))){
					if(!rclcpp::ok()){
						RCLCPP_ERROR(this->get_logger(), "random position client interrupted");
						return;
					} 
				}
							
				//go to point service client
				goto_client = this->create_client<Position>("/go_to_point");
				while (!goto_client -> wait_for_service(std::chrono::seconds(1))){
					if(!rclcpp::ok()){
						RCLCPP_ERROR(this->get_logger(), "go to point client interrupted");
						return;
					} 
				}	
				
			}
			
			
		private:
			bool user_interface(const std::shared_ptr<rmw_request_id_t> request_header,
						const std::shared_ptr<Command::Request> req, 
						const std::shared_ptr<Command::Response> res)
			{
				(void)request_header; // to avoid warnings
				
				if (req->command == "start"){
					this->start = true;
					this->motion();
				}
				else {
					this->start = false;
				}
				res->ok = true;
				return true;
			}
			
			void motion(){
				
				//random position service request
				auto rp_request = std::make_shared<RandomPosition::Request>();
				rp_request->x_max = 5.0;
				rp_request->x_min = -5.0;
				rp_request->y_max = 5.0;
				rp_request->y_min = -5.0;
				
				pos = std::make_shared<Position::Request>();
					
				if (this->start){
					
					//used to synchronize a request for a sevice and use the response
					using ServiceResponseFuture = rclcpp::Client<RandomPosition>::SharedFuture;
						
					// lambda function for Random Position service request 
					auto response_received_callback = [this](ServiceResponseFuture future) {
							
						pos->x = future.get()->x;
						pos->y = future.get()->y;
						pos->theta = future.get()->theta;
						
						std::cout << "\nGoing to the position: x = " << future.get()->x 
								<< " y = " << future.get()->y << " theta = " <<future.get()->theta << std::endl;
										
						//used to synchronize a request for a sevice and use the response
						using ServiceResponseFuture_goto = rclcpp::Client<Position>::SharedFuture;
										
						//lamba function for Go To Point service request
						auto response_received_callback_goto = [this](ServiceResponseFuture_goto future_goto) {
							
							if(future_goto.get()->ok){
								std::cout << "Position reached" << std::endl;
								this->motion();
							}
						};
						
						//request for go to point service
						auto future_result_goto = goto_client->async_send_request(pos, response_received_callback_goto);	
					};
						
					//request for random position service
					auto future_result = rdm_pos_client->async_send_request(rp_request, response_received_callback);
				}
			}		

			
			bool start = true;
			rclcpp::Service<Command>::SharedPtr cmd_service;
			rclcpp::Client<RandomPosition>::SharedPtr rdm_pos_client;
			rclcpp::Client<Position>::SharedPtr goto_client;
			std::shared_ptr<Position::Request> pos;
	};		
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)
