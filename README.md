# ROS/ROS2 Basic Interface

This package contains a practical library to write nodes able to interface with both ROS and ROS2.<br />
The basic idea is to uniform the notation in order to easily switch from ROS to ROS2.
 
### Basic Usage

Inside the header `ros_simple.hpp` is contained the basic interface. To switch between ROS and ROS2 simply change `#define ROS1` in `#define ROS2`.<br />
The new adopted interface is shown, as an example, in `ros_1_test` and `ros_2_test`.

- The class should inherit from `rossimple::Node`.
- The constructor should initialize the inherited class with `rossimple::Node(node_name)`.
- Use `Timer` to create a timer variable.
- Use `Publisher<type>` to create a publisher.
- Use `Subscriber<type>` to create a subscriber.
- Use `Service<type>` to create a service server.
- Use `this->define_timer(m_update_frequency, &timer_callback, this);` to inizialize the timer.
- Use `this->define_publisher<type>("/topic", 10);` to initialize the publisher.
- Use `this->define_subscriber<type>("/topic", 10, &topic_callback, this);` to initialize the subscriber.
- Use `this->define_service<type>("/publish_answer", &service_callback, this);` to initialize the service.
- Use `this->publish<type>(m_publisher, msg);` to publish a message.
- Use `this->get_param<double>("update_frequency", 1.0);` to retrieve a parameter.
- Use `rossimple::init(argc, argv, node_name);` to initialize the node (inside the main).
- Use `rossimple::spin(std::make_shared<class>());` to spin the node (inside the main).
- Use `rossimple::shutdown();` to delete the node (inside the main).
- Use `PRINT_INFO();` to use the ros console.