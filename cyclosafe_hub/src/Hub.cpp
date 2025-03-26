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
#include <fcntl.h>
#include "Hub.hpp"

// using namespace std::chrono_literals;
// using std::chrono::system_clock;
// using std::placeholders::_1;


HubNode::HubNode() : HubNode(rclcpp::Time(0, 0, RCL_ROS_TIME)) {}

HubNode::HubNode(const rclcpp::Time& time_start) :
	Node("hub_node"),
	_sim_start_time(time_start),
	_pending_request_images(nullptr),
	_range_data(std::chrono::milliseconds(0), std::chrono::system_clock::from_time_t(time_start.seconds()) + std::chrono::nanoseconds(time_start.nanoseconds())),
	_gps_data(std::chrono::milliseconds(0), std::chrono::system_clock::from_time_t(time_start.seconds()) + std::chrono::nanoseconds(time_start.nanoseconds()))
{

	this->declare_parameter("cache_ttl", 30.f);
	this->declare_parameter("save_interval", 2.f);
	this->declare_parameter("out_path", "");
	this->declare_parameter("start_time", this->now().seconds());
	
	_cache_ttl = this->get_parameter("cache_ttl").as_double();
	_save_interval = this->get_parameter("save_interval").as_double();
	_paths.main_dir = this->get_parameter("out_path").as_string();
	double received_time = this->get_parameter("start_time").as_double();
	_sim_start_time = rclcpp::Time(int(received_time), 0);
	_range_data.updateTTL(std::chrono::milliseconds(int(_cache_ttl * 1000)));
	_range_data.setStartTime(std::chrono::system_clock::from_time_t(int32_t(_sim_start_time.seconds())));
	_gps_data.updateTTL(std::chrono::milliseconds(int(_cache_ttl * 1000)));
	_gps_data.setStartTime(std::chrono::system_clock::from_time_t(int32_t(_sim_start_time.seconds())));

	if (_paths.main_dir == "" && (_paths.main_dir = this->_get_default_path()) == "")
		throw (HubNode::VerboseException("Could not resolve the default out path ($HOME/data) using $HOME variable"));
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

	RCLCPP_INFO(this->get_logger(), 
			("Hub node initialized and started.\nStart time received=" + 
			std::to_string(_sim_start_time.seconds()) + 
			"\nOut path: " + _paths.main_dir + 
			"\nSave interval: " + std::to_string(_save_interval) + 
			", Cache time-to-live: " + std::to_string(_cache_ttl)).c_str()
	);

}

void	HubNode::_range_callback(const sensor_msgs::msg::Range& msg) {
	system_clock::time_point	timestamp = std::chrono::system_clock::from_time_t(msg.header.stamp.sec) + std::chrono::nanoseconds(msg.header.stamp.nanosec);

	_range_data.push(timestamp, msg);
}


void	HubNode::_gps_callback(const sensor_msgs::msg::NavSatFix& msg) {
	system_clock::time_point	timestamp = std::chrono::system_clock::from_time_t(msg.header.stamp.sec) + std::chrono::nanoseconds(msg.header.stamp.nanosec);

	_gps_data.push(timestamp, msg);
}

void	HubNode::_save_files_callback(void) {
	this->_send_save_images_request();

	_range_data.cleanup();
	_gps_data.cleanup();
	(*_outfile_range) << _range_data;
	_range_data.clear();
	(*_outfile_gps) << _gps_data;
	_gps_data.clear();
}

void	HubNode::_send_save_images_request(void) {

	auto request = std::make_shared<cyclosafe_interfaces::srv::SaveImages::Request>();
	request->path = _paths.images_dir;
	request->time = static_cast<unsigned int>(_save_interval) * 1000U;
	
	if (_pending_request_images) {
		if (_pending_request_images->wait_for(0s) != std::future_status::ready) {
			_nbr_camera_failure++;
			if (_nbr_camera_failure < 3)
				RCLCPP_ERROR(this->get_logger(), "Request to save images timed out. Is the camera node running ?");
		}
		_pending_request_images = nullptr;
	}

	auto res_save_files_callback = [this](rclcpp::Client<cyclosafe_interfaces::srv::SaveImages>::SharedFuture future){
		try {
			auto response = future.get();
			_nbr_camera_failure = 0;
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

std::string	HubNode::_get_default_path(void) {
	const char*	home;

	home = getenv("HOME");
	if (home == NULL)
		return "";
	std::time_t time = std::chrono::system_clock::to_time_t(_sim_start_time.seconds());
	std::tm* timeinfo = std::localtime(&time);
	char buffer[80];
	std::strftime(buffer, sizeof(buffer), "%Y%m%d-%H%M", timeinfo);
	
	return std::string(home) + "/data/" + buffer;
	}

void	HubNode::_setup_out_dir(void) {
	//check if out path exists
	struct stat	file_stats;

	if (stat(_paths.main_dir.c_str(), &file_stats) == 0) { // Check if path already given path exists
		if (S_ISDIR(file_stats.st_mode) == false) // Check if it's a directory
			throw (HubNode::VerboseException(_paths.main_dir + ": not a directory"));
	} else if (mkdir(_paths.main_dir.c_str(), 0711)) // Create the working directory
		throw (HubNode::OSException(_paths.main_dir));

	errno = 0; // discard any previous error
	// // Create or update "latest" symbolic link
	// std::string	symlink_path = _paths.main_dir + "/latest";
	// unlink(symlink_path.c_str());
	// symlink(_paths.main_dir.c_str(), symlink_path.c_str());
	// errno = 0; // discard any unlink() or symlink() error

	if (chdir(_paths.main_dir.c_str())) // Change dir
		throw (HubNode::OSException(_paths.main_dir));
		
	if (mkdir("images", 0711)) // Create images dir
		throw (HubNode::OSException(_paths.main_dir + "/images"));

	_outfile_range = new std::ofstream("range", std::ios_base::app);
	if (_outfile_range == nullptr)
	throw (HubNode::OSException(_paths.main_dir + "/range"));
	
	_outfile_gps = new std::ofstream("gps", std::ios_base::app);
	if (_outfile_gps == nullptr)
		throw (HubNode::OSException(_paths.main_dir + "/gps"));
}

std::ostream&	operator<<(std::ostream& os, const sensor_msgs::msg::Range& msg) {
	os << msg.range;
	return (os);
}

std::ostream&	operator<<(std::ostream& os, const sensor_msgs::msg::NavSatFix& msg) {
	os << msg.latitude << ',' << msg.longitude << ',' << msg.position_covariance[0];
	return (os);
}

HubNode::~HubNode(void) {
	if (this->_outfile_range)
		delete (this->_outfile_range);
	if (this->_outfile_gps)
		delete (this->_outfile_gps);
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

