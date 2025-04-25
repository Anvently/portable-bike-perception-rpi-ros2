#include <string>
#include "Serial.hpp"
#include <sensor_msgs/msg/range.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "BenewakeDriver.hpp"
#include <set>
#include "Utils.hpp"

using namespace std::chrono_literals;

class LidarNode : public rclcpp::Node {
	
	private:

		std::string			_port;
		unsigned int		_baud;
		double				_period;
		unsigned int		_framerate;
		unsigned int		_target_baud;
		bool				_trigger_mode;
		
		// std::deque<unsigned char>								_buffer;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr	_pub;
		rclcpp::TimerBase::SharedPtr							_timer;
		std::shared_ptr<Serial>									_serial;
		std::string												_model;
		std::unique_ptr<Benewake::ADriver>						_driver;
	
		LidarNode(const LidarNode& model);

		void	_retrieve_params(void) {
			this->declare_parameter("port", "/dev/ttyS0");
			this->declare_parameter("baud", 115200);
			this->declare_parameter("period", 0.01);
			this->declare_parameter("model", "");
			this->declare_parameter("trigger", false);
			this->declare_parameter("framerate", 100);
			this->declare_parameter("target_baud", 115200);

			_port = this->get_parameter("port").as_string();
			_model = this->get_parameter("model").as_string();
			_trigger_mode = this->get_parameter("trigger").as_bool();
			
			int baud_int = this->get_parameter("baud").as_int();
			if (baud_int < 0)
				throw InitException("baud must be a positive integer");
			_baud = static_cast<unsigned int>(baud_int);

			int rate_int = this->get_parameter("framerate").as_int();
			if (rate_int < 0)
				throw InitException("framerate must be a positive integer");
			_framerate = static_cast<unsigned int>(rate_int);
			
			int target_baud_int = this->get_parameter("target_baud").as_int();
			if (target_baud_int < 0)
				throw InitException("target_baud must be a positive integer");
			_target_baud = static_cast<unsigned int>(target_baud_int);
		}

		/// @brief 
		/// @param  
		/// @return 0 for success
		int	_configure(void) {
			try {
				if (_target_baud != _baud) {
					_driver->setBaudrate(_target_baud);
					_baud = _target_baud;
				}
				_driver->setFrameRate(_framerate);
				if (_trigger_mode == true)
					_driver->enableOutput();
			} catch (const std::exception& ex) {
				RCLCPP_ERROR(this->get_logger(), ex.what());
				return (1);
			}
			return (0);
		}

		void	_routine(void) {
			Benewake::frames::DataFrame	frame;
			unsigned int				n_received;
			try {
				if (_trigger_mode == true) {
					frame = _driver->triggerDetection();
					if (frame.payload.strength != 65535 && frame.payload.strength >= 100)
						this->_publish(static_cast<double>(frame.payload.distance) / 100.0);
				} else {
					auto start_point = std::chrono::high_resolution_clock::now();
					// Timeout is set to half of period 
					auto timeout = std::chrono::milliseconds(static_cast<unsigned int>((_period / 2.0) * 1000.0)); 
					do {
						frame = _driver->readFrame(1ms, &n_received);
						if (frame.payload.strength != 65535 && frame.payload.strength >= 100)
							this->_publish(static_cast<double>(frame.payload.distance) / 100.0);
					} while (n_received > 0 && Utils::checkTimeout(start_point, timeout) == false);
					if (n_received != 0) {// Means that timeout was detected and some range data likely remain in serial buffer
						RCLCPP_WARN(this->get_logger(), "Failed to read all the data sent by lidar. "\
											"Some readings will be lost. Make sure to set the read period close to the framerate.");
						_serial->flush(); // Flush any remaining readings
					}
				}

			} catch (const std::exception& ex) {
				RCLCPP_ERROR(this->get_logger(), ex.what());
			}

		}

		void	_publish(double distance) {
			auto msg = sensor_msgs::msg::Range();

			msg.header.stamp = this->get_clock()->now();
			msg.field_of_view = _driver->getFov();
			msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
			msg.max_range = msg.min_range = msg.range = distance;
		}

	public:

		LidarNode()
			: Node("lidar_node"), _serial(nullptr) {
			
			_retrieve_params();

			_serial = std::make_shared<Serial>(_port, _baud);
			if (_serial == nullptr)
				throw InitException("Failed to construct serial");

			if (_serial->openSerial())
				throw InitException("Failed to open serial" + std::string(strerror(errno)));
			
			_pub = this->create_publisher<sensor_msgs::msg::Range>("range", rclcpp::QoS(10));
			if (_pub == nullptr)
				throw InitException("Failed to construct publisher");
			
			_timer = this->create_wall_timer(500ms, std::bind(&LidarNode::_routine, this), nullptr, false);
			if (_timer == nullptr)
				throw InitException("Failed to construct timer");

			_driver = Benewake::ADriver::build_driver(_model, _serial);
			if (_driver == nullptr)
				throw InitException("Failed to construct driver from model " + _model);

			if (this->_configure())
				throw InitException("Failed to configure lidar");

			_timer->reset(); // Start the timer routine as autostart was set to false
		}

		virtual ~LidarNode(void) throw() {
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
		rclcpp::spin(std::make_shared<LidarNode>());
		rclcpp::shutdown();
	} catch (const LidarNode::InitException& e) {
		std::cout << e.what() << std::endl;
	}

	return (0);
}
