#ifndef BENEWAKEDRIVER_HPP
# define BENEWAKEDRIVER_HPP

#include <memory>
#include "Serial.hpp"
#include <thread>
#include <cstdint>
#include <string.h>
#include <type_traits>
#include <map>
#include <set>
#include <functional>
#include <algorithm>
#include "Exceptions.hpp"
#include <Utils.hpp>

#define DATA_FRAME_MAGIC 0x5959
#define MAGIC 0x5A

namespace Benewake {

	using namespace std::chrono_literals;
	namespace Ex = Exceptions;

	namespace frames {

		namespace Ex = Exceptions::Frames;

		struct DataFrame {

			#pragma pack(1) // No 10 bytes aligment 
			struct payload {
				uint16_t	magic;
				uint16_t	distance;
				uint16_t	strength;
				uint16_t	temp;
				uint8_t		checksum;
			} payload;

			DataFrame() {}

			DataFrame(char* buffer, unsigned int size) {
				unsigned int sum = 0;

				if (size != sizeof(char[9]))
					throw Ex::SizeException("Invalid data frame size: " + std::to_string(size) + " against expected 9 bytes");
				payload = *(struct payload*)buffer;
				if (payload.magic != DATA_FRAME_MAGIC)
					throw Ex::MagicException("Invalid magic number in data frame: " + std::to_string(payload.magic) + " against " + std::to_string(DATA_FRAME_MAGIC));
				for (unsigned int i = 0; i < sizeof(payload) - 1; i++)
					sum += (unsigned int)buffer[i];
				if (payload.checksum != (uint8_t)(sum & 0xFF))
					throw Ex::ChecksumException("Invalid checksum in data frame: " + std::to_string(payload.checksum) + " in frame instead of " + std::to_string(sum & 0xFF));
			}
		};

		struct CommandFrame {
			const uint8_t magic = 0x5A;
			uint8_t len;
			const uint8_t command_id;
			char* payload = nullptr;
			char* serialized = nullptr;
			uint8_t checksum;

			/// @brief Constructor for integral type
			/// @tparam T any integral type
			/// @param command_id 
			/// @param data 
			/// @param  
			template<typename T>
			CommandFrame(uint8_t command_id, const T& data, typename std::enable_if<std::is_arithmetic<T>::value>::type* = nullptr)
				: len(sizeof(T) + 3 + 1), command_id(command_id)
			{
				payload = new char[sizeof(T)];
				const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);

				for (size_t i = 0; i < sizeof(T); ++i) {
					payload[i] = static_cast<char>(bytes[i]);
				}
				
				calculateChecksum();
				serialized = serialize();
			}

			CommandFrame(uint8_t command_id)
				: len(3 + 1), command_id(command_id)
			{
				calculateChecksum();
				serialized = serialize();
			}

			/// @brief Constructor for array of integral arithmetic types
			/// @tparam T any integral type
			/// @param command_id 
			/// @param data 
			/// @param count 
			/// @param  
			template<typename T>
			CommandFrame(uint8_t command_id, const T* data, size_t count, typename std::enable_if<std::is_arithmetic<T>::value>::type* = nullptr)
				: len(sizeof(T) * count + 3 + 1), command_id(command_id)
			{
				payload = new char[sizeof(T)];
				const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);

				for (size_t i = 0; i < sizeof(T) * count; ++i) {
					payload[i] = static_cast<char>(bytes[i]);
				}
				
				calculateChecksum();
				serialized = serialize();
			}

			~CommandFrame() {
				if (payload)
					delete[] payload;
				if (serialized)
					delete[] serialized;
			}

		private:
			void calculateChecksum() {
				unsigned int sum = (unsigned int)magic + (unsigned int)len + (unsigned int)command_id;
				for (unsigned int i = 0; i < ((unsigned int)len - 3 - 1); i++) {
					sum += (unsigned int)payload[i];
				}
				checksum = (uint8_t)(sum & 0xFF);
			}

			char* serialize(void) const {
				char* out = new char[len];
				out[0] = static_cast<char>(magic);
				out[1] = static_cast<char>(len);
				out[2] = static_cast<char>(command_id);
				memcpy(&out[3], payload, len - 3 - 1);
				out[len - 1] = static_cast<char>(checksum);
				return out;
			}
		};

		struct ResponseFrame {
			uint8_t			magic;
			uint8_t			len;
			uint8_t			command_id;
			char*	payload = nullptr;
			uint8_t			checksum;

			ResponseFrame(char* buffer, unsigned int size, uint8_t expected_id) {
				unsigned int sum = 0;

				if (size < 5)
					throw Ex::SizeException("Invalid frame size in response: " + std::to_string(size) + " against minimum of 5 bytes");

				magic = buffer[0];
				len = buffer[1];
				command_id = buffer[2];
				checksum = buffer[len - 1];

				if (magic != MAGIC) {
					// RCLCPP_DEBUG(this->get_logger(), "%s");
					throw Ex::MagicException("Invalid magic number in response: " + std::to_string(magic) + " against " + std::to_string(MAGIC));
				}
		
				for (uint8_t i = 0; i < (len - 1); i++)
					sum += (unsigned int)buffer[i];

				if (checksum != (uint8_t)(sum & 0xFF))
					throw Ex::ChecksumException("Invalid checksum in response: " + std::to_string(checksum) + " in frame instead of " + std::to_string(sum & 0xFF));

				if (command_id != expected_id)
					throw Ex::CommandMismatchException("Invalid command id in response: " + std::to_string(command_id) + " against expected " + std::to_string(expected_id));

				payload = new char[len - 4];
				memcpy(payload, buffer + 3, len - 4);
			}

			~ResponseFrame() {
				if (payload) {
					delete[] payload;
				}
			}

		};

	}

	class ADriver {

		public:

			using DriverFactory = std::function<std::unique_ptr<ADriver>(std::shared_ptr<Serial>)>;

			static void	setLogger(const rclcpp::Logger& logger) {
				ADriver::_logger = logger;
			}

			static const rclcpp::Logger& getLogger() {
				return _logger;
			}

		private:

			static const std::map<std::string, DriverFactory>	lidar_models;
			static rclcpp::Logger								_logger;


		protected:

			const std::shared_ptr<Serial>	_serial;
			bool							_free_running;

			/// @brief 
			/// @param serial 
			ADriver(std::shared_ptr<Serial> serial) : _serial(serial), _free_running(true) {
				this->detectMode();
			}

			virtual void	setOutput(bool enable) {
				try {
					frames::CommandFrame	command(0x07, static_cast<uint8_t>(enable));
					char					received[5] = {0};
					ssize_t					nbytes;

					RCLCPP_INFO(getLogger(), "%s lidar output", (enable ? "Enabling" : "Disabling"));
					_serial->flush();
					_serial->send(command.serialized, command.len);
					nbytes = _serial->nreceive_peek(received, sizeof(received),  "\x5a\x05", 2, 3 * 1000000);
					if (nbytes < 0)
						throw Ex::SysException("read error");
					else if (nbytes != sizeof(received)) {
						RCLCPP_WARN(_logger, "received %ld bytes, |%.5s|\n", nbytes, received);
						throw Ex::DriverException("timeout");
					}
		
					frames::ResponseFrame	response(received, nbytes, command.command_id);
					if (*(uint8_t*)response.payload != (uint8_t) enable)
						throw Ex::DriverException("invalid response payload");

					_free_running = enable;
					RCLCPP_INFO(getLogger(), "Output %s", (enable ? "enabled" : "disabled"));

				} catch (const std::exception& ex) {
					throw Ex::DriverException("Trying to send enable/disable output command: " + std::string(ex.what()));
				}
			}

		public:

			virtual ~ADriver() = default;

			virtual bool	isFreeRunning() const {
				return _free_running;
			}

			virtual bool	detectMode() {
				ssize_t					nbytes;

				nbytes = _serial->nBytesWaiting();
				if (nbytes > 0)
					_free_running = true;
				else
					_free_running = false;
				return (_free_running);
			}

			/// @brief 
			/// @param timeout 0 to return immediately, -1 to block or > 0 to wait x us
			/// @return 
			template <typename DurationType = std::chrono::milliseconds>
			frames::DataFrame	readFrame(const DurationType& timeout = 1ms) const {
				char					received[9];
				ssize_t					nbytes;

				nbytes = _serial->nreceive_peek(received, sizeof(received), "\x59\x59", 2, std::chrono::duration_cast<std::chrono::microseconds>(timeout).count());
				if (nbytes < 0)
					throw Ex::SysException("read error reading frame");
				else if (nbytes != sizeof(received)) {
					throw Ex::NoDataException();
				}

				frames::DataFrame	response(received, nbytes);

				return response;
			}

			/// @brief 
			/// @param baud 
			virtual void	setBaudrate(unsigned int baud) const {
				unsigned int			previous_baud = _serial->getBaudrate();
				if (_free_running == true)
					throw Ex::DriverException("Trying to config device while it's still free running");

				try {
					frames::CommandFrame	command(0x06, static_cast<uint64_t>(baud));
					char					received[8];
					ssize_t					nbytes;

					RCLCPP_INFO(getLogger(), "Setting baudrate");
					_serial->flush();
					_serial->send(command.serialized, command.len);
					if (_serial->setBaudrate(baud))
						throw Ex::SysException("failed to change serial baudrate");
					std::this_thread::sleep_for(1ms);

					nbytes = _serial->nreceive_peek(received, sizeof(received),  "\x5a\x08", 2, 500 * 1000);
					if (nbytes < 0)
						throw Ex::SysException("read error");
					else if (nbytes != sizeof(received))
						throw Ex::DriverException("timeout reading response");

					frames::ResponseFrame	response(received, nbytes, command.command_id);
					if (*(uint64_t*)response.payload != (uint64_t) baud)
						throw Ex::DriverException("invalid response payload");

					RCLCPP_INFO(getLogger(), "Baudrate set to %u", baud);

				} catch (const std::exception& ex) {
					if (_serial->setBaudrate(previous_baud)) {
						_serial->closeSerial();
						throw Ex::DriverException("Trying to change baudrate (also failed to restore previous baudrate, serial was closed): " + std::string(ex.what()));
					}
					throw Ex::DriverException("Trying to send baudrate command: " + std::string(ex.what()));
				}
			}

			void enableOutput() {
				this->setOutput(true);
			}

			void disableOutput() {
				this->setOutput(false);
			}

			virtual void	saveConfig() const {
				if (_free_running == true)
					throw Ex::DriverException("Trying to config device while it's still free running");

				try {
					frames::CommandFrame	command(0x11);
					char					received[5];
					ssize_t					nbytes;

					_serial->flush();
					_serial->send(command.serialized, command.len);
					std::this_thread::sleep_for(1s);

					nbytes = _serial->nreceive_peek(received, sizeof(received), "\x5a\x05", 2, 0);
					if (nbytes < 0)
						throw Ex::SysException("read error");
					else if (nbytes != sizeof(received)) {
						// RCLCPP_DEBUG
						throw Ex::DriverException("timeout reading response");
					}

					frames::ResponseFrame	response(received, nbytes, command.command_id);
					if (*(uint8_t*)response.payload != 0x00)
						throw Ex::DriverException("unexpected payload response: " + std::to_string(*(uint8_t*)response.payload));

					RCLCPP_INFO(getLogger(), "Config saved");

				} catch (const std::exception& ex) {
					throw Ex::DriverException("Trying to send save settings command: " + std::string(ex.what()));
				}
			}

			virtual void	systemReset() const {
				if (_free_running == true)
					throw Ex::DriverException("Trying to config device while it's still free running");

				try {
					frames::CommandFrame	command(0x10);
					char					received[5];
					ssize_t					nbytes;

					_serial->flush();
					_serial->send(command.serialized, command.len);
					std::this_thread::sleep_for(1s);

					nbytes = _serial->nreceive_peek(received, sizeof(received), "\x5a\x05", 2, 0);
					if (nbytes < 0)
						throw Ex::SysException("read error");
					else if (nbytes != sizeof(received))
						throw Ex::DriverException("timeout reading response");

					frames::ResponseFrame	response(received, nbytes, command.command_id);
					if (*(uint8_t*)response.payload != 0x00)
						throw Ex::DriverException("unexpected response payload, received: " + std::to_string(*(uint8_t*)response.payload));

					RCLCPP_INFO(getLogger(), "System reset");

				} catch (const std::exception& ex) {
					throw Ex::DriverException("Trying to send factory reset command: " + std::string(ex.what()));
				}
			}

			virtual frames::DataFrame	triggerDetection() const {
				if (_free_running == true)
					throw Ex::DriverException("Trying to trigger detection while device is in free running mode");

				try {
					frames::CommandFrame	command(0x04);

					_serial->flush();
					_serial->send(command.serialized, command.len);
					std::this_thread::sleep_for(20ms);

					return this->readFrame();

				} catch (const std::exception& ex) {
					throw Ex::DriverException("Trying to send trigger detection command: " + std::string(ex.what()));
				}
			}

			/// @brief Set the output framerate
			/// @param rate 
			/// @note For ```TFS20-L``` device, possible value include ```0, 20, 50, 100, 250```
			/// Actual rate will be the first ```value >= rate```
			/// @note For ```TF02-Pro``` device, max value is ```1000Hz```
			virtual void	setFrameRate(unsigned int rate) const = 0;

			virtual std::string getModelName() const = 0;

			virtual double	getFov() const {
				return 1.0;
			}

			static std::unique_ptr<ADriver> build_driver(const std::string& model_name, std::shared_ptr<Serial> serial_port) {
				auto it = lidar_models.find(model_name);
				if (it != lidar_models.end()) {
					return it->second(serial_port);
				}
				return nullptr;
			}

	};

	class TF02Driver : public ADriver {

		public:

			TF02Driver(std::shared_ptr<Serial> serial) : ADriver(serial) {

			}

			virtual std::string getModelName() const {return ("tf-02");}
			virtual double	getFov() const {return (3.0);}

			virtual void	setFrameRate(unsigned int rate) const override {
				if (_free_running == true)
					throw Ex::DriverException("Trying to config device while it's still free running");
				try {
					char					received[6];
					ssize_t					nbytes;
					frames::CommandFrame	command(0x03, static_cast<uint16_t>(rate));

					RCLCPP_INFO(getLogger(), "Setting frame rate to %u", rate);

					if (rate > 1000)
						throw Ex::DriverException("parameter error => framerate must be less than 1000Hz: " + std::to_string(rate));
			
					_serial->flush();
					_serial->send(command.serialized, command.len);

					nbytes = _serial->nreceive_peek(received, sizeof(received), "\x5a\x06", 2, 3 * 1000000);
					if (nbytes < 0)
						throw Ex::SysException("read error");
					else if (nbytes != sizeof(received))
						throw Ex::DriverException("timeout reading response");
						
					frames::ResponseFrame	response(received, nbytes, command.command_id);
					if (*(uint16_t*)response.payload != (uint16_t) rate)
						throw Ex::DriverException("invalid response payload");

					RCLCPP_INFO(getLogger(), "Frame rate set to %u hertz", rate);

				} catch (const std::exception& ex) {
					throw Ex::DriverException("Trying to set framerate: " + std::string(ex.what()));
				}
			}
	};

	class TFMiniPlusDriver : public ADriver {

		public:

			TFMiniPlusDriver(std::shared_ptr<Serial> serial) : ADriver(serial) {

			}

			virtual std::string getModelName() const {return ("tf-mini-plus");}
			virtual double	getFov() const {return (3.6);}

			virtual void	setFrameRate(unsigned int rate) const override {

				RCLCPP_INFO(getLogger(), "Setting frame rate to %u", rate);
				if (_free_running == true)
					throw Ex::DriverException("Trying to config device while it's still free running");
				try {
					char					received[6];
					ssize_t					nbytes;
					frames::CommandFrame	command(0x03, static_cast<uint16_t>(rate));

					if (rate > 1000)
						throw Ex::DriverException("parameter error => framerate must be less than 1000Hz: " + std::to_string(rate));
			
					_serial->flush();
					_serial->send(command.serialized, command.len);

					nbytes = _serial->nreceive_peek(received, sizeof(received), "\x5a\x06", 2, 3 * 1000000);
					if (nbytes < 0)
						throw Ex::SysException("read error");
					else if (nbytes != sizeof(received))
						throw Ex::DriverException("timeout reading response");
				
					frames::ResponseFrame	response(received, nbytes, command.command_id);
					if (*(uint16_t*)response.payload != static_cast<uint16_t>(rate)) {
						throw Ex::DriverException("invalid response payload");
					}

					RCLCPP_INFO(getLogger(), "Frame rate was set to %u", rate);

				} catch (const std::exception& ex) {
					throw Ex::DriverException("Trying to set framerate: " + std::string(ex.what()));
				}
			}
	};

	class TFS20LDriver : public ADriver {

		private:

			static const std::set<unsigned int>	rate_values;

			static bool	isValidFramerate(unsigned int rate) {
				return (rate_values.find(rate) != rate_values.end());
			}

			static unsigned int	match_rate_value(unsigned int rate) {
				if (rate_values.find(rate) != rate_values.end())
					return rate;
				auto it = rate_values.upper_bound(rate);
				if (it == rate_values.end())
					return (*rate_values.lower_bound(rate));
				return (*it);
			}

		public:

			TFS20LDriver(std::shared_ptr<Serial> serial) : ADriver(serial) {

			}

			virtual std::string getModelName() const {return ("tf-s20l");}
			virtual double	getFov() const {return (2.0);}

			virtual void	setFrameRate(unsigned int rate) const override {
				if (_free_running == true)
					throw Ex::DriverException("Trying to config device while it's still free running");
				try {
					char					received[6];
					ssize_t					nbytes;
					frames::CommandFrame	command(0x04, static_cast<uint16_t>(match_rate_value(rate)));


					if (rate > 250)
						throw Ex::DriverException("invalid parameter => must be less than 250Hz: " + std::to_string(rate));
			
					_serial->flush();
					_serial->send(command.serialized, command.len);

					nbytes = _serial->nreceive_peek(received, sizeof(received), "\x5a\x06", 2, 3 * 1000000);
					if (nbytes < 0)
						throw Ex::SysException("read error");
					else if (nbytes != sizeof(received))
						throw Ex::DriverException("timeout reading response");
						
					frames::ResponseFrame	response(received, nbytes, command.command_id);
					if (*(uint16_t*)response.payload != (uint16_t) rate)
						throw Ex::DriverException("invalid response payload");

				} catch (const std::exception& ex) {
					throw Ex::DriverException("Trying to send enable/disable output command: " + std::string(ex.what()));
				}
			}
	};

}

rclcpp::Logger Benewake::ADriver::_logger = rclcpp::get_logger("default_logger_name");

const std::set<unsigned int> Benewake::TFS20LDriver::rate_values = {0, 20, 50, 100, 250};

const std::map<std::string, Benewake::ADriver::DriverFactory> Benewake::ADriver::lidar_models = {
	{"tf-s20l", [](std::shared_ptr<Serial> serial) { return std::make_unique<Benewake::TFS20LDriver>(serial); }},
	{"tf-mini-plus", [](std::shared_ptr<Serial> serial) { return std::make_unique<Benewake::TFMiniPlusDriver>(serial); }},
	{"tf-02", [](std::shared_ptr<Serial> serial) { return std::make_unique<Benewake::TF02Driver>(serial); }}
};


#endif