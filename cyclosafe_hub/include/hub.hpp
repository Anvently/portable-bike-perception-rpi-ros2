#include <string>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cyclosafe_interfaces/srv/save_images.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <ctime>
#include <chrono>
#include <format>
#include <sys/stat.h>
#include <unistd.h>

using namespace std::chrono_literals;
using std::chrono::system_clock;
using std::placeholders::_1;

class Datas {

	private:

		

	public:

};

namespace utils {

	std::string	get_default_path(void);
}

class HubNode : public rclcpp::Node {
	
	private:
		
		double							_cache_ttl; // Data time-to-live
		double							_save_interval; // Seconds
		struct {
			std::string	parent_dir; // outpath given as parameter. Ex: ~/data/
			std::string	main_dir; // name of the run. Ex: ~/data/20240321-1318
			std::string	images_dir;
		}	_paths;

		system_clock::time_point		_sim_start_t;

		rclcpp::TimerBase::SharedPtr	_timer_save_files;

		rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr		_sub_range;
		rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr	_sub_gps;
		rclcpp::Client<cyclosafe_interfaces::srv::SaveImages>::SharedPtr	_client_images;

		using SaveImagesFutureResponse = rclcpp::Client<cyclosafe_interfaces::srv::SaveImages>::SharedFutureAndRequestId;
		std::shared_ptr<SaveImagesFutureResponse>	_pending_request_images;

		void	_range_callback(const sensor_msgs::msg::Range& msg);

		void	_gps_callback(const sensor_msgs::msg::NavSatFix& msg);

		void	_save_files_callback(void);

		void	_setup_out_dir(void);

		HubNode(const HubNode& model);

	public:

		HubNode();


		virtual ~HubNode(void) throw() {
			
		}


	class VerboseException : public std::exception {
		private:
			const std::string	message;
		public:
			VerboseException(const std::string& message) : message(message) {}
			virtual ~VerboseException(void) throw() {}
			virtual const char*	what(void) const throw() {return (this->message.c_str());}
	};

	class OSException : public std::exception {
		private:
			const std::string	message;
			const int			error;
		public:
			OSException(const std::string& message) : message(std::format(
				"{0}: {1} ({2})", message.c_str(), strerror(errno), errno)), error(errno) {
			}
			virtual ~OSException(void) throw() {}
			virtual const char*	what(void) const throw() {
				return (message.c_str());
			}
	};
	
};

