#include "rclcpp.hpp"
#include <string>


class LidarNode : public rclcpp::Node {
	
	private:

		const std::string	_port;
		const int			_baudrate;
		
		int					_fd;
		std::deque<char>	_buffer;
	
		LidarNode(const LidarNode& model);
		// virtual ~LidarNode();

		int	openSerial(void);
		int	closeSerial(void);

	public:

		LidarNode::LidarNode(const std::string& port, int baudrate) : _port(port), _baudrate(baudrate), Node("lidar_node") {
				
		}

		

	class InitException : public std::exception {
		private:
			const std::string	message;
		public:
			InitException(const std::string& message) : message(message) {}
			virtual ~InitException(void) throw() {}
			virtual const char*	what(void) const throw() {return (this->message.c_str());}
	}
	
};

int	main(int argc, char** argv) {


	return (0);
}
