#include "Utils.hpp"
#include <string>
#include <iostream>
#include <iomanip>

void	Utils::printReceived(size_t nbytes, char* received) {
	std::cout << nbytes << "|";
	for (unsigned int i = 0; i < nbytes; ++i)
		std::cout << std::hex << std::setfill('0') << std::setw(2) << ((unsigned int)received[i] & 0xFF) << " ";
	std::cout << std::endl;
}