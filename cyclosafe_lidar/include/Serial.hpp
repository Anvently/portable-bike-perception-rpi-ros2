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
		std::deque<char>	_buffer;

		static ssize_t	_read(int fd, char* buffer, size_t n);

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
		
		/// @brief Read everything until timeout
		/// @param dest destination string
		/// @param timeout in us, 0 to return immediately or > 0 to wait x us
		/// @return number of bytes read, 0 if timeout, -errno if error
		ssize_t	receive(std::string& dest, unsigned long timeout);
	
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
		ssize_t		receive(char* buffer, size_t nmax, bool block = false);

		/// @brief Read serial
		/// @param buffer Char deque storing what's read
		/// @param nmax Read a maximum of nbytes
		/// @param block Block until something is read, else return immediately
		/// @return Number of bytes read or -errno if error
		ssize_t		receive(std::deque<char>& dest, size_t nmax, bool block = false);

		/// @brief Block as long as nbytes were not received
		/// @param buffer destination buffer
		/// @param n number of bytes to receive
		/// @param timeout in us, 0 to return immediately, -1 to block or > 0 to wait x us
		/// @return number of bytes read, 0 if timeout, -errno if error
		ssize_t		nreceive(char* buffer, size_t n, long timeout);

		/// @brief Block as long as peek_len (starting from a given delimiter peek) were not received
		/// and timeout is not exceeded
		/// @param dest
		/// @param len_dest 
		/// @param peek delimiter char sequence
		/// @param peek_len len of delimiter sequence
		/// @param timeout in us, 0 to return immediately, -1 to block or > 0 to wait x us
		/// @return 
		ssize_t		nreceive_peek(char* dest, size_t len_dest, const char* peek, size_t peek_len, long timeout);

		// ssize_t		read_until()

		int			send(const char* str, size_t n);
		int			send(const std::string& str);
		int			send(unsigned char byte);

		size_t		nBytesWaiting(void) const;
		bool		isOpen(void) const;


};

#endif
