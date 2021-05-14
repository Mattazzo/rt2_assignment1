/**
 * Class StateMachine in namespace rt2_assignment1, it implements a node 
 * with server for user_interface service, if user request "start" the node 
 * asks random postition service for a random goal and then call go to point 
 * service to reach the goal.
 */
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

namespace rt2_assignment1{
	
	/**
	 * Class StateMachine that implement robot behaviour, implement a server 
	 * for user_command service, when user request to start the robot, the 
	 * class call his method motion() to let it request a random goal and 
	 * reach it. If user request to stop the robot, it will stop when the
	 * next goal is reached. 
	 */
	class StateMachine : public rclcpp::Node
	{
		public:
			
			/**
			 * Class constructor, it contains declaration of a server for 
			 * Command service, a client for RandomPosition and a client for 
			 * Position service
			 * 
			 * @param 
			 * 		options: NodeOptions object to create the node
			 */
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
			/**
			 * Function of server for Command service. if user request "start" 
			 * the method motio() is called and robot request and reach a goal,
			 * otherwise user request is "stop" and robot is stopped when
			 * the next goal is reached
			 * 
			 * @param
			 * 		request_header: parameter to avoid warnings
			 * 		req: request for the Command service, could be "start" or "stop"
			 * 		res: response is true to confirm that server is working
			 * 
			 * @return
			 * 		True
			 */
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
			
			/**
			 * Method to request a random goal for the robotusing RandomPosition 
			 * service and reach it calling Position service
			 */
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

			
			bool start = true;											// var used to know when robot should move
			rclcpp::Service<Command>::SharedPtr cmd_service;			// server for user interface service
			rclcpp::Client<RandomPosition>::SharedPtr rdm_pos_client;	// client for random position service
			rclcpp::Client<Position>::SharedPtr goto_client;			// client for go to point service
			std::shared_ptr<Position::Request> pos;						// request for go to point service
	};		
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)
