#include "Serial.hpp"
#include <string>
#include <deque>
#include <iostream>
#include <algorithm>

std::string	_extractSep(std::deque<unsigned char>& _buffer, char separator) {

	// size_t	len_str;
	
	auto pos_substr = std::find(_buffer.begin(), _buffer.end(), separator);
	if (pos_substr == _buffer.end())
		return "";
	// len_str = pos_substr - _buffer.begin();
	std::string	substr(_buffer.begin(), std::next(pos_substr));
	// std::cout << "len_str=" << pos_substr - _buffer.begin() << std::endl;
	_buffer.erase(_buffer.begin(), std::next(pos_substr));
	return (substr);
}

int main(int argc, char** argv) {

	if (argc != 2)
		return (0);
	Serial serial(argv[1], 115200);
	if (serial.openSerial())
		return(1);
	std::deque<unsigned char>	my_deque;
	while (1) {
		serial.receive(my_deque, 1024, true);
		// std::cout << std::string(my_deque.begin(), my_deque.end());
		std::string	str;
		while ((str = _extractSep(my_deque, '\n')) != "") {
			std::cout << str;
		}

	}
	return(0);
}