#include <string>
#include <string.h>

namespace Exceptions {
	class VerboseException : public std::exception {
			protected:
				const std::string	message;
			public:
				VerboseException(const std::string& message) : message(message) {}
				virtual ~VerboseException(void) throw() {}
				virtual const char*	what(void) const throw() {return (this->message.c_str());}
	};

	class SysException : public VerboseException {
			protected:
				const std::string	message;
			public:
				SysException(const std::string& message) : VerboseException(
					message + ": " + strerror(errno) + " (" + std::to_string(errno) + ")") {}
	};

	namespace Frames {
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

	class DriverException : public VerboseException {
			public:
				DriverException(const std::string& message) : VerboseException(message) {}
	};

	class NoDataException : public std::exception {
			public:
				NoDataException() {}
	};
}