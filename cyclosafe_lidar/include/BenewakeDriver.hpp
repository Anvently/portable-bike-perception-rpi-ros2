#ifndef BENEWAKEDRIVER_HPP
# define BENEWAKEDRIVER_HPP

#include <memory>
#include "Serial.hpp"
#include <thread>
#include <cstdint>
#include <string.h>
#include <type_traits>

#define DATA_FRAME_MAGIC 0x5959
#define MAGIC 0x5A

namespace BenewakeDriver {

	namespace frames {

		struct DataFrame {

			#pragma pack(1) // No 10 bytes aligment 
			struct payload {
				uint16_t	magic;
				uint16_t	distance;
				uint16_t	strength;
				uint16_t	temp;
				uint8_t		checksum;
			} payload;

			DataFrame(char* buffer, unsigned int size) {
				unsigned int sum;

				if (size != sizeof(char[9]))
					throw SizeException("Invalid data frame size: " + std::to_string(size) + " against expected 9 bytes");
				payload = *(struct payload*)buffer;
				if (payload.magic != DATA_FRAME_MAGIC)
					throw MagicException("Invalid magic number in data frame: " + std::to_string(payload.magic) + " against " + std::to_string(DATA_FRAME_MAGIC));
				for (unsigned int i = 0, sum = 0; i < sizeof(payload) - 1; i++)
					sum += (unsigned char)buffer[i];
				if (payload.checksum != (uint8_t)(sum & 0xFF))
					throw ChecksumException("Invalid checksum in data frame: " + std::to_string(payload.checksum) + " in frame instead of " + std::to_string(sum & 0xFF));
			}
		};

		struct CommandFrame {
			const uint8_t magic = 0x5A;
			uint8_t len;
			const uint8_t command_id;
			std::string payload;
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
				payload.resize(sizeof(T));
				const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&data);

				for (size_t i = 0; i < sizeof(T); ++i) {
					payload[i] = static_cast<char>(bytes[i]);
				}
				
				calculateChecksum();
			}

			CommandFrame(uint8_t command_id)
				: len(3 + 1), command_id(command_id)
			{
				calculateChecksum();
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
				payload.resize(sizeof(T) * count);
				const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);

				for (size_t i = 0; i < sizeof(T) * count; ++i) {
					payload[i] = static_cast<char>(bytes[i]);
				}
				
				calculateChecksum();
			}

			std::string toString(void) const {
				std::string result;
				result.reserve(len);
				result.push_back(static_cast<char>(magic));
				result.push_back(static_cast<char>(len));
				result.push_back(static_cast<char>(command_id));
				result.append(payload);
				result.push_back(static_cast<char>(checksum));
				return result;
			}

		private:
			void calculateChecksum() {
				unsigned int sum = magic + len + command_id;
				for (std::string::const_iterator it = payload.begin(); it != payload.end(); it++) {
					sum += (unsigned char)*it;
				}
				checksum = (uint8_t)(sum & 0xFF);
			}
		};

		struct ResponseFrame {
			uint8_t			magic;
			uint8_t			len;
			uint8_t			command_id;
			char*	payload = nullptr;
			uint8_t			checksum;

			ResponseFrame(char* buffer, unsigned int size, uint8_t expected_id) {
				unsigned int sum;

				if (size < 5)
					throw SizeException("Invalid frame size in response: " + std::to_string(size) + " against minimum of 5 bytes");

				magic = buffer[0];
				len = buffer[1];
				command_id = buffer[2];
				checksum = buffer[len - 1];

				if (magic != DATA_FRAME_MAGIC)
					throw MagicException("Invalid magic number in response: " + std::to_string(magic) + " against " + std::to_string(DATA_FRAME_MAGIC));
		
				for (unsigned int i = 0, sum = 0; i < len - 1; i++)
					sum += (unsigned char)buffer[i];
				if (checksum != (uint8_t)(sum & 0xFF))
					throw ChecksumException("Invalid checksum in response: " + std::to_string(checksum) + " in frame instead of " + std::to_string(sum & 0xFF));

				if (command_id != expected_id)
					throw CommandMismatchException("Invalid command id in response: " + std::to_string(command_id) + " against expected " + std::to_string(expected_id));

				payload = new char[len - 4];
				memcpy(payload, buffer + 3, len - 4);
			}

			~ResponseFrame() {
				if (payload) {
					delete[] payload;
				}
			}

		};

		class SizeException : public VerboseException {
			public:
				SizeException(const std::string& message) : VerboseException(message) {}
		};

		class MagicException : public VerboseException {
			public:
				MagicException(const std::string& message) : VerboseException(message) {}
		};

		class ChecksumException : public VerboseException {
			public:
				ChecksumException(const std::string& message) : VerboseException(message) {}
		};

		class CommandMismatchException : public VerboseException {
			public:
				CommandMismatchException(const std::string& message) : VerboseException(message) {}
		};
	}

	class ADriver {

		protected:

			const std::shared_ptr<Serial>	_serial;
			bool							_free_running;

			/// @brief 
			/// @param serial 
			ADriver(std::shared_ptr<Serial> serial) : _serial(serial), _free_running(true) {
				
			}

			virtual int	setOutput(bool enable) {
				try {
					frames::CommandFrame	command(0x07, static_cast<uint8_t>(enable));
					char					received[5];
					ssize_t					nbytes;

					_serial->flush();
					_serial->send(command.toString());
					std::this_thread::sleep_for(std::chrono::milliseconds(1));

					nbytes = _serial->nreceive((unsigned char*)received, 5, 1 * 1000000);
					if (nbytes < 0)
						throw SysException("Trying to send set output command");
					else if (nbytes != 5)
						throw DriverException("Timeout when trying to send output command");
						
					frames::ResponseFrame	response(received, nbytes, command.command_id);
					if (*(uint8_t*)response.payload != (uint8_t) enable)
						throw DriverException("Invalid response payload when trying to switch output");

					nbytes = _serial->nreceive((unsigned char*)received, 5, 500 * 1000);
					if (nbytes != 0)
						throw DriverException("Device output was disable but it's still sending data");

					_free_running = enable;

				} catch (const std::exception& ex) {
					throw DriverException("Trying to send enable/disable output command: " + std::string(ex.what()));
				}
				

			}

		public:

			virtual ~ADriver(void) = 0;

			/// @brief 
			/// @param baud 
			/// @return 0 for success, 1 for error, or -1 if sensor is free running
			virtual void	setBaudrate(unsigned int baud) const {
				unsigned int			previous_baud = _serial->getBaudrate();
				if (_free_running == true)
					throw DriverException("Trying to config device while it's still free running");

				try {
					frames::CommandFrame	command(0x06, static_cast<uint64_t>(baud));
					char					received[8];
					ssize_t					nbytes;

					_serial->flush();
					_serial->send(command.toString());
					if (_serial->setBaudrate(baud));
						throw SysException("Trying to set serial baudrate");
					std::this_thread::sleep_for(std::chrono::milliseconds(1));

					nbytes = _serial->nreceive((unsigned char*)received, sizeof(received), 500 * 1000);
					if (nbytes < 0)
						throw SysException("Trying to send baudrate command");
					else if (nbytes != sizeof(received))
						throw DriverException("Timeout when trying to send baudrate command");

					frames::ResponseFrame	response(received, nbytes, command.command_id);
					if (*(uint64_t*)response.payload != (uint64_t) baud)
						throw DriverException("Invalid response payload when sending change baudrate command");

				} catch (const std::exception& ex) {
					if (_serial->setBaudrate(previous_baud))
						_serial->closeSerial();
						throw DriverException("Trying to change baudrate (also failed to restore previous baudrate, serial was closed): " + std::string(ex.what()));
					throw DriverException("Trying to send baudrate command: " + std::string(ex.what()));
				}
			}

			int enableOutput() {
				return (this->setOutput(true));
			}

			int enableOutput() {
				return (this->setOutput(false));
			}

			virtual void	saveConfig() const {
				if (_free_running == true)
					throw DriverException("Trying to config device while it's still free running");

				try {
					frames::CommandFrame	command(0x11);
					char					received[5];
					ssize_t					nbytes;

					_serial->flush();
					_serial->send(command.toString());
					std::this_thread::sleep_for(std::chrono::seconds(1));

					nbytes = _serial->nreceive((unsigned char*)received, sizeof(received), 0);
					if (nbytes < 0)
						throw SysException("Trying to send save settings command");
					else if (nbytes != sizeof(received))
						throw DriverException("Timeout when trying to send save settings command");

					frames::ResponseFrame	response(received, nbytes, command.command_id);
					if (*(uint8_t*)response.payload != 0x00)
						throw DriverException("Invalid response payload when sending save settings command. Received: " + std::to_string(*(uint8_t*)response.payload));

				} catch (const std::exception& ex) {
					throw DriverException("Trying to send save settings command: " + std::string(ex.what()));
				}
			}

			virtual void	systemReset() const {
				if (_free_running == true)
					throw DriverException("Trying to config device while it's still free running");

				try {
					frames::CommandFrame	command(0x10);
					char					received[5];
					ssize_t					nbytes;

					_serial->flush();
					_serial->send(command.toString());
					std::this_thread::sleep_for(std::chrono::seconds(1));

					nbytes = _serial->nreceive((unsigned char*)received, sizeof(received), 0);
					if (nbytes < 0)
						throw SysException("Trying to send factory reset command");
					else if (nbytes != sizeof(received))
						throw DriverException("Timeout when trying to send factory reset command");

					frames::ResponseFrame	response(received, nbytes, command.command_id);
					if (*(uint8_t*)response.payload != 0x00)
						throw DriverException("Invalid response payload when sending factory reset command. Received: " + std::to_string(*(uint8_t*)response.payload));

				} catch (const std::exception& ex) {
					throw DriverException("Trying to send factory reset command: " + std::string(ex.what()));
				}
			}

			virtual frames::DataFrame	triggerDetection() const {
				if (_free_running == true)
					throw DriverException("Trying to trigger detection while device is in free running mode");

				try {
					frames::CommandFrame	command(0x04);
					char					received[9];
					ssize_t					nbytes;

					_serial->flush();
					_serial->send(command.toString());

					nbytes = _serial->nreceive((unsigned char*)received, sizeof(received), 5 * 1000); //5ms timeout
					if (nbytes < 0)
						throw SysException("Trying to send trigger detection command");
					else if (nbytes != sizeof(received))
						throw DriverException("Timeout when trying to send trigger detection command");

					frames::DataFrame	response(received, nbytes);

					return response;

				} catch (const std::exception& ex) {
					throw DriverException("Trying to send trigger detection command: " + std::string(ex.what()));
				}
			}

			virtual void	setFrameRate() const;

	};

	class TF02Driver : public ADriver {

		public:

			TF02Driver(std::shared_ptr<Serial> serial) : ADriver(serial) {

			}



	};

	class VerboseException : public std::exception {
			protected:
				const std::string	message;
			public:
				VerboseException(const std::string& message) : message(message) {}
				virtual ~VerboseException(void) throw() {}
				virtual const char*	what(void) const throw() {return (this->message.c_str());}
	};

	class DriverException : public VerboseException {
			public:
				DriverException(const std::string& message) : VerboseException(message) {}
	};

	class SysException : public VerboseException {
			protected:
				const std::string	message;
			public:
				SysException(const std::string& message) : VerboseException(
					message + ": " + strerror(errno) + " (" + std::to_string(errno) + ")") {}
	};

}


#endif