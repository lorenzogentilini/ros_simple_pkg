#include "ros_simple/ros_simple.hpp" // Include the ROS Simplifier

// User Messages
#include "std_msgs/String.h"
#include "std_srvs/SetBool.h"

static const std::string node_name = "simple_subscriber_test";

class SimpleSubscriber : public rossimple::Node
{
	public:
		SimpleSubscriber() : rossimple::Node(node_name)
		{
			m_subscriber = this->define_subscriber<std_msgs::String>("/topic", 10, &SimpleSubscriber::topic_callback, this);
			m_service = this->define_service<std_srvs::SetBool>("/publish_answer", &SimpleSubscriber::service_callback, this);
		}

	private:
		void topic_callback(const std_msgs::String::ConstPtr& msg)
		{
			PRINT_INFO(this, "I heard: '%s'", msg->data.c_str());
		}

		bool service_callback(std_srvs::SetBool::Request& request,
							  std_srvs::SetBool::Response& response)
		{
			if(request.data){
				PRINT_INFO(this, "Good Boy!!");
			} else{
				PRINT_INFO(this, "Bad Boy!!");
			}

			return true;
		}

		Subscriber<std_msgs::String> m_subscriber;
		Service<std_srvs::SetBool> m_service;
};

int main(int argc, char* argv[])
{
	rossimple::init(argc, argv, node_name);
	rossimple::spin(std::make_shared<SimpleSubscriber>());
	rossimple::shutdown();
	return 0;
}
