#include <string>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cyclosafe_interfaces/srv/save_images.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

using namespace std::chrono_literals;

class Datas {

	private:

		

	public:

};

class HubNode : public rclcpp::Node {
	
	private:
		
		double							_cache_tll; // Data time-to-live
		double							_save_interval; // Seconds
		typedef struct {
			std::string	parent_dir; // outpath given as parameter. Ex: ~/data/
			std::string	main_dir; // name of the run. Ex: ~/data/20240321-1318
			std::string	images_dir; // subfolder containing images. Ex: ~/data/20240321-1318/images/
		}	_paths;

		rclcpp::TimerBase::SharedPtr	_timer_save_images;

		rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr		_sub_range;
		rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr	_sub_gps;
		rclcpp::Client<cyclosafe_interfaces::srv::SaveImages>::SharedPtr	_client_images;


		HubNode(const HubNode& model);

	public:

		HubNode() : Node("hub_node") {

			this->declare_parameter("cache_ttl", 30.f);
			this->declare_parameter("save_interval", 10.f);
			this->declare_parameter("save_path", "");

			_cache_tll = this->get_parameter("cache_ttl").as_double();
			_save_interval = this->get_parameter("_save_interval").as_double();

			_serial = std::make_shared<Serial>(port, baudrate);
			if (_serial == nullptr)
				throw InitException("Failed to construct serial");

			if (_serial->openSerial())
				throw InitException("Failed to open serial" + std::string(strerror(errno)));
			
			_pub = this->create_publisher<sensor_msgs::msg::Range>("lidar_range", rclcpp::QoS(10));
			if (_pub == nullptr)
				throw InitException("Failed to construct publisher");
			
			_timer = this->create_wall_timer(500ms, std::bind(&HubNode::_routine, this));
			if (_timer == nullptr)
				throw InitException("Failed to construct timer");
		}

		virtual ~HubNode(void) throw() {
			if (_serial && _serial->isOpen())
				_serial->closeSerial();
		}


	class InitException : public std::exception {
		private:
			const std::string	message;
		public:
			InitException(const std::string& message) : message(message) {}
			virtual ~InitException(void) throw() {}
			virtual const char*	what(void) const throw() {return (this->message.c_str());}
	};
	
};

int	main(int argc, char** argv) {

	try {
		rclcpp::init(argc, argv);
		rclcpp::spin(std::make_shared<HubNode>());
		rclcpp::shutdown();
	} catch (const HubNode::InitException& e) {
		std::cout << e.what() << std::endl;
	}

	return (0);
}
