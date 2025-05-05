#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include "Serial.hpp"
// #include <termio.h>
#include <asm/termbits.h>
// #include "termios.h"
#include <string.h>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <Utils.hpp>
#include <sys/select.h>

/// @brief Find needle sequence in haystack
/// @param haystack 
/// @param hay_len 
/// @param needle 
/// @param needle_len 
/// @return nullptr if needle sequence not found in haystack
static const char*	find_sequence(const char* haystack, size_t hay_len, const char* needle, size_t needle_len) {
	unsigned int i, j;

	for (i = 0; i < hay_len; i++) {
		for (j = 0; j < needle_len && i + j < hay_len && (unsigned char)haystack[i + j] == (unsigned char)needle[j]; j++);
		if (j == needle_len)
			return (haystack + i);
	}
	return (nullptr);
}

Serial::Serial(const std::string& port, const int baud) : _port(port), _baud(baud){
	_fd = -1;
}

// Serial::Serial(const Serial& copy) : _port(""), _baud(0) {}

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
	memset(&tio, 0, sizeof(tio));
	tio.c_cflag = CS8 | CLOCAL | CREAD;
	tio.c_iflag = IGNPAR;
	tio.c_cc[VMIN] = 0;
	tio.c_cc[VTIME] = 10;
	ioctl(_fd, TCGETS, &tio); //Set initial flags
	
	ioctl(_fd, TCGETS2, &tio2);
	tio2.c_cflag &= ~CBAUD;
	tio2.c_cflag |= BOTHER;
	tio2.c_ospeed = _baud;
	tio2.c_ispeed = _baud;
	ioctl(_fd, TCSETS2, &tio2);

	usleep(100 * 1000);

	flush();
	return (0);
}

bool	Serial::isOpen(void) const {
	return (_fd >= 0);
}

int Serial::setBaudrate(unsigned int baud) {
	if (_fd < 0) {
		_baud = baud;
		return (0);
	}
	
	struct termios2 tio2;

	if (ioctl(_fd, TCGETS2, &tio2) < 0)
		return errno;
	
	tio2.c_cflag &= ~CBAUD;
	tio2.c_cflag |= BOTHER;
	tio2.c_ospeed = baud;
	tio2.c_ispeed = baud;
	
	if (ioctl(_fd, TCSETS2, &tio2) < 0)
		return errno;
	
	_baud = baud;

	ioctl(_fd, TCFLSH, TCIOFLUSH);
	
	return 0;
}

unsigned int	Serial::getBaudrate(void) const {
	return (_baud);
}

void	Serial::flush(void) const {
	ioctl(_fd, TCFLSH, TCIOFLUSH);
}

ssize_t	Serial::_read(int fd, char* buffer, size_t n) {
	fd_set fds;
	struct timeval	timeout = {0, 0};
	int	ret = -1;

	FD_ZERO(&fds);
	FD_SET(fd, &fds);
	if (select(fd + 1, &fds, 0, 0,&timeout) == 1) {/* there is data available */
		ret = read(fd, buffer, n);
	} else {
		errno = EWOULDBLOCK;
	}
	return (ret);
}

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

ssize_t	Serial::receive(std::string& dest, unsigned long timeout) {
	ssize_t	total = 0;
	ssize_t nbytes;
	char	buffer[1024];
	auto start_time = std::chrono::high_resolution_clock::now();

	while (1) {
		nbytes = read(_fd, &buffer[0], 1023);
		if (nbytes < 0) {
			if (errno != EAGAIN)
				return (-errno);
		}
		else if (nbytes > 0) {
			dest += std::string(buffer, nbytes);
			total += nbytes;
		} else {
			this->closeSerial();
		}
		if (Utils::checkTimeout(start_time, std::chrono::microseconds(timeout)) == true)
			break;
		usleep(100);
	}
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
			throw std::exception();
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
ssize_t	Serial::receive(char* buffer, size_t nmax, bool block) {
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

ssize_t		Serial::receive(std::deque<char>& dest, size_t nmax, bool block) {
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

ssize_t		Serial::nreceive(char* buffer, size_t n, long timeout) {
	ssize_t	ret;
	size_t	nread = 0;
	auto start_time = std::chrono::high_resolution_clock::now();

	while (nread < n) {
		ret = read(_fd, buffer + nread, n - nread);
		if (ret < 0) {
			if (errno == EAGAIN) {
				if (timeout == 0)
					break;
				if (timeout < 0 || Utils::checkTimeout(start_time, std::chrono::microseconds(timeout)) == false)
					continue;
				break;
			}
			return (-errno);
		}
		if (ret == 0) {
			this->closeSerial();
		} else {
			nread += ret;
			if (nread == n)
				break;
		}
		usleep(100);
	}
	return (nread);
}

ssize_t		Serial::nreceive_peek(char* dest, size_t len_dest, const char* peek, size_t peek_len, long timeout) {
	ssize_t	ret, len;
	size_t	nread = 0;
	auto start_time = std::chrono::high_resolution_clock::now();
	char	tmp[1024] = {0};

	while (nread < len_dest) {
		ret = read(_fd, tmp, 1024);
	
		if (ret < 0) {
			if (errno == EAGAIN) {
				if (timeout == 0 || (timeout > 0 && Utils::checkTimeout(start_time, std::chrono::microseconds(timeout)) == true))
					break;
				errno = 0;
			} else {
				return (-errno);
			}
		}
		else if (ret == 0) {
			this->closeSerial();
		} else {
			if (nread == 0) {
				const char* pos = find_sequence(tmp, ret, peek, peek_len);
				if (pos != nullptr) {
					len = std::min((size_t)ret - (pos - tmp), len_dest);
					memcpy(dest, pos, len);
					nread += len;
				}
			} else {
				len = std::min((size_t)ret, len_dest - nread);
				memcpy(dest + nread, tmp, len);
				nread += len;
			}
		}
	}
	usleep(100);
	return (nread);
}



int	Serial::send(const char* str, size_t n) {
	int ret;

	ret = write(_fd, str, n);
	if (ret < 0)
		std::cerr << "Write error" << std::endl;
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
