#include <string>
#include "Serial.hpp"
#include <sensor_msgs/msg/range.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

using namespace std::chrono_literals;



class LidarNode : public rclcpp::Node {
	
	private:

		// const std::string	_port;
		// const int			_baudrate;
		
		std::deque<unsigned char>								_buffer;
		rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr	_pub;
		rclcpp::TimerBase::SharedPtr							_timer;
		std::shared_ptr<Serial>									_serial;
	
		LidarNode(const LidarNode& model);

		void	_routine(void) {
			
			// _buffer.resize(_buffer.size() + _serial->nBytesWaiting());

			if (_serial->receive(_buffer, _serial->nBytesWaiting(), true) < 0) {
				std::cout << "Error: " << errno << ": " << strerror(errno) << std::endl;
			}
			
			if (_buffer.size())
				std::cout << "Deque size=" << _buffer.size() << std::endl;

			std::string line;
			while ((line = _extractSep('\n')) != "") {
				std::cout << line;
			}

		}

		std::string	_extractSep(char separator) {

			auto pos_substr = std::find(_buffer.begin(), _buffer.end(), separator);
			if (pos_substr == _buffer.end())
				return "";
			// len_str = pos_substr - _buffer.begin();
			std::string	substr(_buffer.begin(), std::next(pos_substr));
			_buffer.erase(_buffer.begin(), std::next(pos_substr));
			return (substr);
		}

	public:

		LidarNode()
			: Node("lidar_node"), _serial(nullptr) {

			std::string	port;
			int			baudrate;
			
			this->declare_parameter("port", port);
			this->declare_parameter("baudrate", 9600);

			port = this->get_parameter("port").as_string();
			baudrate = this->get_parameter("baudrate").as_int();

			_serial = std::make_shared<Serial>(port, baudrate);
			if (_serial == nullptr)
				throw InitException("Failed to construct serial");

			if (_serial->openSerial())
				throw InitException("Failed to open serial" + std::string(strerror(errno)));
			
			_pub = this->create_publisher<sensor_msgs::msg::Range>("lidar_range", rclcpp::QoS(10));
			if (_pub == nullptr)
				throw InitException("Failed to construct publisher");
			
			_timer = this->create_wall_timer(500ms, std::bind(&LidarNode::_routine, this));
			if (_timer == nullptr)
				throw InitException("Failed to construct timer");
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
