#ifndef SERIAL_HPP
# define SERIAL_HPP

#include <string>
#include <deque>
#include <chrono>

class Serial {

	private:

		Serial(const Serial& copy);

		const std::string			_port;
		int							_baud;
		std::deque<unsigned char>	_buffer;

		int	_fd;
	
	public:

		Serial(const std::string& port, const int baud);
		virtual ~Serial(void);

		int	openSerial(void);
		int	closeSerial(void);

		int	setBaudrate(unsigned int baud);
		unsigned int	getBaudrate(void) const;

		void	flush(void) const;

		/// @brief Read serial and store everything in dest
		/// @param dest 
		/// @return Number of bytes read or -errno if error
		/// @note No Timeout
		ssize_t		receive(std::string& dest);
		
		/// @brief Read serial and return content read
		/// @param dest 
		/// @return Number of bytes read or -errno if error
		/// @note No Timeout
		std::string	receive(void);

		/// @brief Read serial
		/// @param buffer Buffer storing what will be read
		/// @param nmax Read a maximum of nbytes
		/// @param block Block until something is read, else return immediately
		/// @return Number of bytes read or -errno if error
		ssize_t		receive(unsigned char* buffer, size_t nmax, bool block = false);

		/// @brief Read serial
		/// @param buffer Char deque storing what's read
		/// @param nmax Read a maximum of nbytes
		/// @param block Block until something is read, else return immediately
		/// @return Number of bytes read or -errno if error
		ssize_t		receive(std::deque<unsigned char>& dest, size_t nmax, bool block = false);

		/// @brief Block as long as nbytes were not received
		/// @param buffer destination buffer
		/// @param n number of bytes to receive
		/// @param timeout in us, 0 to return immediately, -1 to block or > 0 to wait x us
		/// @return 
		ssize_t		nreceive(unsigned char* buffer, size_t n, int timeout);

		// ssize_t		read_until()

		int			send(const char* str, size_t n);
		int			send(const std::string& str);
		int			send(unsigned char byte);

		size_t		nBytesWaiting(void) const;
		bool		isOpen(void) const;

		static bool	checkTimeout(const std::chrono::time_point<std::chrono::high_resolution_clock>& start_time, unsigned long timeout_us);

};

#endif
