#include "ros_simple/ros_simple.hpp" // Include the ROS Simplifier

// User Messages
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

static const std::string node_name = "simple_subscriber_test";

class SimpleSubscriber : public rossimple::Node
{
	public:
		SimpleSubscriber() : rossimple::Node(node_name)
		{
			m_subscriber = this->define_subscriber<std_msgs::msg::String>("/topic", 10, &SimpleSubscriber::topic_callback, this);
			m_service = this->define_service<std_srvs::srv::SetBool>("/publish_answer", &SimpleSubscriber::service_callback, this);
		}

	private:
		void topic_callback(const std::shared_ptr<std_msgs::msg::String> msg)
		{
			PRINT_INFO(this, "I heard: '%s'", msg->data.c_str());
		}

		void service_callback(const std::shared_ptr<Request<std_srvs::srv::SetBool>> request,
									std::shared_ptr<Response<std_srvs::srv::SetBool>> response)
		{
			if(request->data){
				PRINT_INFO(this, "Good Boy!!");
			} else{
				PRINT_INFO(this, "Bad Boy!!");
			}
		}

		Subscriber<std_msgs::msg::String> m_subscriber;
		Service<std_srvs::srv::SetBool> m_service;

		int count = 0;
};

int main(int argc, char* argv[])
{
	rossimple::init(argc, argv, node_name);
	rossimple::spin(std::make_shared<SimpleSubscriber>());
	rossimple::shutdown();
	return 0;
}
