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
#include "hub.hpp"

// using namespace std::chrono_literals;
// using std::chrono::system_clock;
// using std::placeholders::_1;


namespace utils {

	std::string	get_default_path(void) {
		const char*	home;

		home = getenv("HOME");
		if (home == NULL)
			return "";
		return std::format("{0}/data", home);
	}

}

HubNode::HubNode() : Node("hub_node"), _pending_request_images(nullptr)
		{

			this->declare_parameter("cache_ttl", 30.f);
			this->declare_parameter("save_interval", 2.f);
			this->declare_parameter("save_path", "");
			
			_cache_ttl = this->get_parameter("cache_ttl").as_double();
			_save_interval = this->get_parameter("save_interval").as_double();
			_paths.parent_dir = this->get_parameter("save_path").as_string();
			
			if (_paths.parent_dir == "" && (_paths.parent_dir = utils::get_default_path()) == "")
				throw (HubNode::VerboseException("Could not resolve the default out path ($HOME/data) using $HOME variable"));
			_sim_start_t = system_clock::now();
			_paths.main_dir = std::format("{0}/{1:%Y%m%d-%H%M}", _paths.parent_dir, _sim_start_t);
			_paths.images_dir = _paths.main_dir + "/images";
			this->_setup_out_dir();

			_sub_range = this->create_subscription<sensor_msgs::msg::Range>("range", 10, std::bind(&HubNode::_range_callback, this, _1));
			if (_sub_range == nullptr)
				throw HubNode::VerboseException("Failed to construct range suscriber");

			_sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps", 10, std::bind(&HubNode::_gps_callback, this, _1));
			if (_sub_gps == nullptr)
				throw HubNode::VerboseException("Failed to construct gps suscriber");

			_client_images = this->create_client<cyclosafe_interfaces::srv::SaveImages>("save_images");
			
			_timer_save_files = this->create_wall_timer(std::chrono::duration<double>(_save_interval), std::bind(&HubNode::_save_files_callback, this));
			if (_timer_save_files == nullptr)
				throw HubNode::VerboseException("Failed to construct timer");

			RCLCPP_INFO(this->get_logger(), std::format(
				"Hub node initialized and started.\nOut path: {0}\nSave interval: {1}, Cache time-to-live: {2}", _paths.main_dir, _save_interval, _cache_ttl
			).c_str());

		}

void	HubNode::_range_callback(const sensor_msgs::msg::Range& msg) {
	(void)msg;
}

void	HubNode::_gps_callback(const sensor_msgs::msg::NavSatFix& msg) {
	(void)msg;
}

void	HubNode::_save_files_callback(void) {

	auto request = std::make_shared<cyclosafe_interfaces::srv::SaveImages::Request>();
	request->path = _paths.images_dir;
	request->time = _save_interval;
	
	if (_pending_request_images) {
		if (_pending_request_images->wait_for(0s) != std::future_status::ready)
			RCLCPP_ERROR(this->get_logger(), "Request to save images timed out. Is the camera node running ?");
		_pending_request_images = nullptr;
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
	

	_pending_request_images = std::make_shared<SaveImagesFutureResponse>(_client_images->async_send_request(request, res_save_files_callback));

}

void	HubNode::_setup_out_dir(void) {
	//check if out path exists
	struct stat	file_stats;

	if (stat(_paths.parent_dir.c_str(), &file_stats)) // Check if given path exists
		throw (HubNode::OSException(_paths.parent_dir));

	if (S_ISDIR(file_stats.st_mode) == false) // Check if it's a directory
		throw (HubNode::VerboseException(std::format("{0}: not a directory", _paths.parent_dir.c_str())));

	if (mkdir(_paths.main_dir.c_str(), 0711)) // Create the working directory
		throw (HubNode::OSException(_paths.main_dir));	

	// Create or update "latest" symbolic link
	std::string	symlink_path = _paths.parent_dir + "/latest";
	unlink(symlink_path.c_str());
	symlink(_paths.main_dir.c_str(), symlink_path.c_str());
	errno = 0; // discard any unlink() or symlink() error

	if (chdir(_paths.main_dir.c_str())) // Change dir
		throw (HubNode::OSException(_paths.main_dir));
		

	if (mkdir("images", 0711)) // Create images dir
		throw (HubNode::OSException(_paths.main_dir + "/images"));

}


int	main(int argc, char** argv) {

	try {
		rclcpp::init(argc, argv);
		rclcpp::spin(std::make_shared<HubNode>());
		rclcpp::shutdown();
	} catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
	}

	return (0);
}
