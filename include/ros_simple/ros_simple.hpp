#pragma once // First Guard

#ifndef _ROS_SIMPLE_ // Second Guard
#define _ROS_SIMPLE_

#define ROS1 // Define the Framework

// Redefine Types for ROS1
#ifdef ROS1
#include "ros/ros.h"

template<typename T>
using Publisher = ros::Publisher;

template<typename T>
using Subscriber = ros::Subscriber;

template<typename T>
using Service = ros::ServiceServer;

template<typename T>
using Request = typename T::Request;

template<typename T>
using Response = typename T::Response;

#define Timer ros::Timer
#endif

// Redefine Types for ROS2
#ifdef ROS2
#include <chrono>
#include "rclcpp/rclcpp.hpp"

template<typename T>
using Publisher = typename rclcpp::Publisher<T>::SharedPtr;

template<typename T>
using Subscriber = typename rclcpp::Subscription<T>::SharedPtr;

template<typename T>
using Service = typename rclcpp::Service<T>::SharedPtr;

template<typename T>
using Request = typename T::Request;

template<typename T>
using Response = typename T::Response;

#define Timer rclcpp::TimerBase::SharedPtr
#endif

namespace rossimple{
// Basic ROS Functions
void init(int argc, char* argv[], std::string node_name)
{
	#ifdef ROS1
	ros::init(argc, argv, node_name);
	#endif
	#ifdef ROS2
	rclcpp::init(argc, argv);
	#endif
}

void shutdown()
{
	#ifdef ROS1
	// Nothing
	#endif
	#ifdef ROS2
	rclcpp::shutdown();
	#endif
}

template <typename F>
void spin(F&& fun)
{
	#ifdef ROS1
	ros::spin();
	#endif
	#ifdef ROS2
	rclcpp::spin(fun);
	#endif
}

// Node Class Functions
class Node 
	#ifdef ROS2
	: public rclcpp::Node
	#endif
	{

	public:
	// Public Functions
	Node(std::string node_name);
	~Node();

	// Parameter Handler
	template <typename T>
	T get_param(std::string param_name, T default_value)
	{
		T param;

		#ifdef ROS1
		nh.param(param_name, param, default_value);
		#endif

		#ifdef ROS2
		this->declare_parameter(param_name, default_value);
        param = this->get_parameter(param_name).get_parameter_value().get<T>();
		#endif

		return param;
	}

	// Define Publisher
	template <typename T>
	Publisher<T> define_publisher(std::string topic_name, int queue_lenght)
	{
		#ifdef ROS1
		return nh.advertise<T>(topic_name, queue_lenght);
		#endif

		#ifdef ROS2
		return this->create_publisher<T>(topic_name, queue_lenght);
		#endif
	}

	// Define Subscriber
	template <typename T, typename F1, typename F2>
	Subscriber<T> define_subscriber(std::string topic_name, int queue_lenght, F1&& bind_function, F2&& object_pointer)
	{
		#ifdef ROS1
		return nh.subscribe(topic_name, queue_lenght, bind_function, object_pointer);
		#endif

		#ifdef ROS2
		return this->create_subscription<T>(topic_name, queue_lenght, std::bind(bind_function, object_pointer, std::placeholders::_1));
		#endif
	}

	// Define Timer
	template <typename F1, typename F2>
	Timer define_timer(double update_frequency, F1&& bind_function, F2&& object_pointer)
	{
		#ifdef ROS1
		return nh.createTimer(ros::Duration(1/update_frequency), std::bind(bind_function, object_pointer));
		#endif

		#ifdef ROS2
		return this->create_wall_timer(std::chrono::nanoseconds((int)(1e9/update_frequency)), std::bind(bind_function, object_pointer));
		#endif
	}

	// Define Service
	template <typename T, typename F1, typename F2>
	Service<T> define_service(std::string service_name, F1&& bind_function, F2&& object_pointer)
	{
		#ifdef ROS1
		return nh.advertiseService(service_name, bind_function, object_pointer);
		#endif

		#ifdef ROS2
		return this->create_service<T>(service_name, std::bind(bind_function, object_pointer, std::placeholders::_1, std::placeholders::_2));
		#endif
	}

	// Publishing Function
	template <typename T>
	void publish(Publisher<T> publisher, T msg)
	{
		#ifdef ROS1
		publisher.publish(msg);
		#endif

		#ifdef ROS2
		publisher->publish(msg);
		#endif
	}

	// Public Attributes
	#ifdef ROS1
	ros::NodeHandle nh;
	#endif
};

// Logger Functions
template <typename F, typename... Args>
void PRINT_INFO(F&& object_pointer, const char* str, Args... args)
{
	#ifdef ROS1
	ROS_INFO(str, args...);
	#endif

	#ifdef ROS2
	RCLCPP_INFO(object_pointer->get_logger(), str, args...);
	#endif
}

template <typename F, typename... Args>
void PRINT_WARN(F&& object_pointer, const char* str, Args... args)
{
	#ifdef ROS1
	ROS_WARN(str, args...);
	#endif

	#ifdef ROS2
	RCLCPP_WARN(object_pointer->get_logger(), str, args...);
	#endif
}

template <typename F, typename... Args>
void PRINT_ERROR(F&& object_pointer, const char* str, Args... args)
{
	#ifdef ROS1
	ROS_ERROR(str, args...);
	#endif

	#ifdef ROS2
	RCLCPP_ERROR(object_pointer->get_logger(), str, args...);
	#endif
}
} // End Namespace rossimple
#endif
