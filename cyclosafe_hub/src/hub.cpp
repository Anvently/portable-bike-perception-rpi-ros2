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

	std::string	get_default_path(void) {
		const char*	home;

		home = getenv("HOME");
		if (home == NULL)
			return "";
		return std::format("{0}/data", home);
	}

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

		using SaveImagesFutureResponse = rclcpp::Client<cyclosafe_interfaces::srv::SaveImages>::SharedFuture;
		std::shared_ptr<SaveImagesFutureResponse>	_pending_request_images;

		void	_range_callback(const sensor_msgs::msg::Range& msg) {
			(void)msg;
		}

		void	_gps_callback(const sensor_msgs::msg::NavSatFix& msg) {
			(void)msg;
		}

		void	_save_files_callback(void) {

			auto request = std::make_shared<cyclosafe_interfaces::srv::SaveImages::Request>();
			request->path = _paths.images_dir;
			request->time = _save_interval;
			
			if (_pending_request_images) {
				if (_pending_request_images->wait_for(0s) != std::future_status::ready)
					RCLCPP_ERROR(this->get_logger(), "Request to save images timed out. Is the camera node running ?");
				_pending_request_images.reset();
			}

			auto res_save_files_callback = [this](rclcpp::Client<cyclosafe_interfaces::srv::SaveImages>::SharedFuture future){
				try {
					auto response = future.get();
					if (response->result == response->PARTIAL_SUCCESS)
						RCLCPP_WARN(this->get_logger(), "Camera node could not fulfill the timespan requirement. Some images will be missing.");
					if (response->result == response->FAILURE)
						RCLCPP_ERROR(this->get_logger(), "Camera node failed to save images. Check storage usage");
				} catch (const std::exception& e) {
					RCLCPP_ERROR(this->get_logger(), "Runtime error: reading response from save images service.");
				}
				_pending_request_images = nullptr;
			};
			

			auto result = _client_images->async_send_request(request);

		}

		void	_setup_out_dir(void) {
			//check if out path exists
			struct stat	file_stats;

			if (_paths.parent_dir == "" && (_paths.parent_dir = utils::get_default_path()) == "")
				throw (VerboseException("Could not resolve the default out path ($HOME/data) using $HOME variable"));

			if (stat(_paths.parent_dir.c_str(), &file_stats)) // Check if given path exists
				throw (OSException(_paths.parent_dir));

			if (S_ISDIR(file_stats.st_mode) == false) // Check if it's a directory
				throw (VerboseException(std::format("{0}: not a directory", _paths.parent_dir.c_str())));

			if (mkdir(_paths.main_dir.c_str(), 0711)) // Create the working directory
				throw (OSException(_paths.main_dir));

			if (chdir(_paths.main_dir.c_str())) // Change dir
				throw (OSException(_paths.main_dir));

			if (mkdir("images", 0711)) // Create images dir
				throw (OSException(_paths.main_dir + "/images"));

		}

		HubNode(const HubNode& model);

		rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_; // The subscriber object.

	public:

		HubNode() : Node("hub_node"), _pending_request_images(nullptr)
		{

			this->declare_parameter("cache_ttl", 30.f);
			this->declare_parameter("save_interval", 10.f);
			this->declare_parameter("save_path", "");
			
			_cache_ttl = this->get_parameter("cache_ttl").as_double();
			_save_interval = this->get_parameter("save_interval").as_double();
			_paths.parent_dir = this->get_parameter("save_path").as_string();
			
			_sim_start_t = system_clock::now();
			_paths.main_dir = std::format("{0}/{:%Y%m%d-%H%M}", _paths.parent_dir, _sim_start_t);
			_paths.images_dir = _paths.main_dir + "/images";
			this->_setup_out_dir();

			_sub_range = this->create_subscription<sensor_msgs::msg::Range>("range", 10, std::bind(&HubNode::_range_callback, this, _1));
			if (_sub_range == nullptr)
				throw VerboseException("Failed to construct range suscriber");

			_sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps", 10, std::bind(&HubNode::_gps_callback, this, _1));
			if (_sub_gps == nullptr)
				throw VerboseException("Failed to construct gps suscriber");

			_client_images = this->create_client<cyclosafe_interfaces::srv::SaveImages>("save_images");
			
			_timer_save_files = this->create_wall_timer(500ms, std::bind(&HubNode::_save_files_callback, this));
			if (_timer_save_files == nullptr)
				throw VerboseException("Failed to construct timer");

			RCLCPP_INFO(this->get_logger(), std::format(
				"Hub node initialized and started.\nOut path: {0}\nSave interval: {1}, Cache time-to-live: {2}", _paths.main_dir, _save_interval, _cache_ttl
			));

		}


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
			OSException(const std::string& message) : message(message), error(errno) {}
			virtual ~OSException(void) throw() {}
			virtual const char*	what(void) const throw() {
				return (std::format(
						"{0}: {1} ({2})", this->message.c_str(), strerror(error), error)
					).c_str();
			}
	};
	
};

int	main(int argc, char** argv) {

	try {
		rclcpp::init(argc, argv);
		rclcpp::spin(std::make_shared<HubNode>());
		rclcpp::shutdown();
	} catch (const HubNode::VerboseException& e) {
		std::cout << e.what() << std::endl;
	}

	return (0);
}
