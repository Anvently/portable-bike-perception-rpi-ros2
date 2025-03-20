#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include "Serial.hpp"
// #include <termio.h>
#include <asm/termbits.h>
// #include "termios.h"
#include <string.h>
#include <format>
#include <iostream>
#include <algorithm>

Serial::Serial(const std::string& port, const int baudrate) : _port(port), _baudrate(baudrate){
	_fd = -1;
}

// Serial::Serial(const Serial& copy) : _port(""), _baudrate(0) {}

Serial::~Serial(void) {
	this->closeSerial();
}

int	Serial::closeSerial(void) {
	if (_fd >= 0)
		ioctl(_fd, TCFLSH, TCIOFLUSH);
	close(_fd);
	return (0);
}

int	Serial::openSerial(void) {
	struct termios	tio;
	struct termios2	tio2;

	_fd = open(_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (_fd < 0)
		return (errno);
	if (isatty(_fd) != 1)
		return (-1);
	tio.c_cflag |= CS8 | CLOCAL | CREAD;
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 1;
	ioctl(_fd, TCGETS, &tio); //Set initial flags
	
	ioctl(_fd, TCGETS2, &tio2);
	tio2.c_cflag &= ~CBAUD;
	tio2.c_cflag |= BOTHER;
	tio2.c_ospeed = _baudrate;
	tio2.c_ispeed = _baudrate;
	ioctl(_fd, TCSETS2, &tio2);

	ioctl(_fd, TCFLSH, TCIOFLUSH);
	return (0);
}

bool	Serial::isOpen(void) const {
	return (_fd >= 0);
}

/// @brief Read everything until there's nothing left to read and adds it to dest.
/// @param dest 
/// @return Number of bytes read, or < 0 for error
ssize_t	Serial::receive(std::string& dest) {
	ssize_t	total = 0;
	ssize_t nbytes;
	char	buffer[1024];

	do {
		nbytes = read(_fd, &buffer[0], 1023);
		if (nbytes < 0) {
			if (errno == EAGAIN)
				break;
			return (-errno);
		}
		else if (nbytes > 0) {
			dest += std::string(buffer, nbytes);
			total += nbytes;
		} else {
			this->closeSerial();
		}
	} while (nbytes > 0);
	return (total);
}

std::string	Serial::receive(void) {
	ssize_t 	nbytes;
	std::string	message;
	char		buffer[1024];

	do {
		nbytes = read(_fd, &buffer[0], 1023);
		if (nbytes < 0) {
			if (errno == EAGAIN)
				break;
			return std::string(std::format("error: {0}: {1}", errno, strerror(errno)));
		}
		else if (nbytes > 0) {
			message += std::string(buffer, nbytes);
		} else {
			this->closeSerial();
		}
	} while (nbytes > 0);
	return (message);
}

/// @brief Read up to ```nmax``` bytes from serial to ```buffer```
/// @param buffer 
/// @param nmax 
/// @param block defines if an empty serial buffer should block until ```nmax```
/// bytes are actually read.
/// @return 
ssize_t	Serial::receive(unsigned char* buffer, size_t nmax, bool block) {
	ssize_t	ret;
	size_t	nread = 0;

	while (nread < nmax) {
		ret = read(_fd, buffer + nread, nmax - nread);
		if (ret < 0) {
			if (errno == EAGAIN) {
				if (block == true)
					continue;
				break;
			}
			return (-errno);
		}
		if (ret == 0) {
			this->closeSerial();
		} else {
			nread += ret;
		}
	}
	return (nread);
}

ssize_t		Serial::receive(std::deque<unsigned char>& dest, size_t nmax, bool block) {
	ssize_t	ret;
	size_t	nread = 0;
	char	buffer[1024];

	while (nread < nmax) {
		ret = read(_fd, &buffer[0], std::min(nmax - nread, 1024UL));
		if (ret < 0) {
			if (errno == EAGAIN) {
				if (block == true)
					continue;
				break;
			}
			return (-errno);
		}
		if (ret == 0) {
			this->closeSerial();
		} else {
			nread += ret;
			std::copy(&buffer[0], &buffer[ret], std::back_insert_iterator(dest));
		}
	}
	return (nread);
}

int	Serial::send(const char* str, size_t n) {
	int ret;

	ret = write(_fd, str, n);
	return (ret);
}

int Serial::send(const std::string& message) {
	int	ret;

	ret = write(_fd, message.c_str(), message.length());
	return (ret);
}

int	Serial::send(unsigned char byte) {
	int ret;

	ret = write(_fd, &byte, 1);
	return (ret);
}

size_t	Serial::nBytesWaiting(void) const {
	size_t	nbytes = 0;

	if (isOpen() == false)
		return (0);
	ioctl(_fd, FIONREAD, &nbytes);
	return (nbytes);
}

