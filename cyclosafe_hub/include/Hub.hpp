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
#include <TTLDeque.hpp>
#include <ostream>
#include <fstream>

using namespace std::chrono_literals;
using std::chrono::system_clock;
using std::placeholders::_1;


namespace utils {

	std::string	get_default_path(void);
}

std::ostream&	operator<<(std::ostream& os, sensor_msgs::msg::Range);
std::ostream&	operator<<(std::ostream& os, sensor_msgs::msg::NavSatFix);

class HubNode : public rclcpp::Node {
	
	private:
		
		HubNode();

		double							_cache_ttl; // Data time-to-live
		double							_save_interval; // Seconds
		struct {
			std::string	parent_dir; // outpath given as parameter. Ex: ~/data/
			std::string	main_dir; // name of the run. Ex: ~/data/20240321-1318
			std::string	images_dir;
		}	_paths;
		
		std::ofstream*	_outfile_range;
		std::ofstream*	_outfile_gps;

		system_clock::time_point		_sim_start_time;

		rclcpp::TimerBase::SharedPtr	_timer_save_files;

		using Range = sensor_msgs::msg::Range;
		using NavSatFix = sensor_msgs::msg::NavSatFix;

		rclcpp::Subscription<Range>::SharedPtr		_sub_range;
		rclcpp::Subscription<NavSatFix>::SharedPtr	_sub_gps;
		rclcpp::Client<cyclosafe_interfaces::srv::SaveImages>::SharedPtr	_client_images;

		using SaveImagesFutureResponse = rclcpp::Client<cyclosafe_interfaces::srv::SaveImages>::SharedFutureAndRequestId;
		std::shared_ptr<SaveImagesFutureResponse>	_pending_request_images;

		TTLDeque<Range>	_range_data;
		TTLDeque<NavSatFix>	_gps_data;

		void	_range_callback(const sensor_msgs::msg::Range& msg);

		void	_gps_callback(const sensor_msgs::msg::NavSatFix& msg);

		void	_save_files_callback(void);
		void	_send_save_images_request(void);

		void	_setup_out_dir(void);

		HubNode(const HubNode& model);
		HubNode(system_clock::time_point time_start);
		HubNode();

	public:


		virtual ~HubNode(void) throw();


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

