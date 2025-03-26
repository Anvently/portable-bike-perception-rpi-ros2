#include <string>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cyclosafe_interfaces/srv/save_images.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <ctime>
#include <chrono>
#include <sys/stat.h>
#include <unistd.h>
#include <TTLDeque.hpp>
#include <ostream>
#include <fstream>
// #include <rclcpp/time.hpp>

using namespace std::chrono_literals;
using std::chrono::system_clock;
using std::placeholders::_1;

std::ostream&	operator<<(std::ostream& os, const sensor_msgs::msg::Range&);
std::ostream&	operator<<(std::ostream& os, const sensor_msgs::msg::NavSatFix&);

class HubNode : public rclcpp::Node {
	
	private:

		double							_cache_ttl; // Data time-to-live
		double							_save_interval; // Seconds
		struct {
			std::string	main_dir; // name of the run. Ex: ~/data/20240321-1318
			std::string	images_dir;
		}	_paths;
		
		std::ofstream*	_outfile_range;
		std::ofstream*	_outfile_gps;

		rclcpp::Time					_sim_start_time;

		rclcpp::TimerBase::SharedPtr	_timer_save_files;
		unsigned int					_nbr_camera_failure = 0;

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

		std::string	_get_default_path(void);
		void		_setup_out_dir(void);

		HubNode(const HubNode& model);
		
	public:
		
		HubNode();
		HubNode(const rclcpp::Time& time_start);

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
			OSException(const std::string& message)
				: message(message + ": " + strerror(errno) + " (" + std::to_string(errno)),
				error(errno)
				{}
			virtual ~OSException(void) throw() {}
			virtual const char*	what(void) const throw() {
				return (message.c_str());
			}
	};
	
};

