#include "ros_simple/ros_simple.hpp" // Include the ROS Simplifier

// User Messages
#include "std_msgs/String.h"

static const std::string node_name = "simple_publisher_test";

class SimplePublisher : public rossimple::Node
{
	public:
		SimplePublisher() : rossimple::Node(node_name)
		{
			m_update_frequency = this->get_param<double>("update_frequency", 1.0);
			m_publisher = this->define_publisher<std_msgs::String>("/topic", 10);
			m_timer = this->define_timer(m_update_frequency, &SimplePublisher::timer_callback, this);
		}

	private:
		void timer_callback()
		{
			std_msgs::String msg;
			msg.data = "Hello, World! " + std::to_string(m_count++);

			PRINT_INFO(this, "Publishing: '%s'", msg.data.c_str());
			this->publish<std_msgs::String>(m_publisher, msg);
		}

		Timer m_timer;
		Publisher<std_msgs::String> m_publisher;

		int m_count = 0;
		double m_update_frequency = 1;
};

int main(int argc, char* argv[])
{
	rossimple::init(argc, argv, node_name);
	rossimple::spin(std::make_shared<SimplePublisher>());
	rossimple::shutdown();
	return 0;
}
