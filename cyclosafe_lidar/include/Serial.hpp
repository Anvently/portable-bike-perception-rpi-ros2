#ifndef SERIAL_HPP
# define SERIAL_HPP

#include <string>
#include <deque>

class Serial {

	private:

		Serial(const Serial& copy);

		const std::string			_port;
		const int					_baudrate;
		std::deque<unsigned char>	_buffer;

		int	_fd;
	
	public:

		Serial(const std::string& port, const int baudrate);
		virtual ~Serial(void);

		int	openSerial(void);
		int	closeSerial(void);

		ssize_t		receive(std::string& dest);
		std::string	receive(void);
		ssize_t		receive(unsigned char* buffer, size_t nmax, bool block = false);
		ssize_t		receive(std::deque<unsigned char>& dest, size_t nmax, bool block = false);
		int			send(const char* str, size_t n);
		int			send(const std::string& str);
		int			send(unsigned char byte);

		size_t		nBytesWaiting(void) const;
		bool		isOpen(void) const;

};

#endif
