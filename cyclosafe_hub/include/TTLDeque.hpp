#include <deque>
#include <chrono>
#include <algorithm>
#include <ostream>

/*
TTL based container.
Store pairs of time and any data type 
ttl check is done at each add

*/
template <typename T>
class TTLDeque {

	private:

		using TimePoint = std::chrono::system_clock::time_point;
		using Item = std::pair<std::chrono::milliseconds, T>;

		std::deque<Item>			_container;
		std::chrono::milliseconds	_ttl;
		TimePoint					_start;
		
		/// @brief Remove expired element from the container
		
		std::chrono::milliseconds	_time() {
			return (std::chrono::system_clock::now() - _start);
		}
		
	public:
	
		TTLDeque(std::chrono::milliseconds ttl, TimePoint start) : _ttl(ttl), _start(start) {}

		TTLDeque(std::chrono::milliseconds ttl) : _ttl(ttl) {
			_start = std::chrono::system_clock::now();
		}

		void	cleanup() {
			TimePoint	expires_before(_time() - _ttl);
	
			auto upper = std::upper_bound(_container.begin(), _container.end(), expires_before);
			if (upper == _container.end())
				return;
			std::erase(_container.begin(), upper);
		}
		
		void	push(const TimePoint& at,  const T& item) {
			_container.emplace_back(at - _start, item);
		}

		void	updateTTL(std::chrono::milliseconds ttl) {
			_ttl = ttl;
		}

		bool	empty() const {
			return (_container.size() == 0);
		}

		size_t	size() const {
			return _container.size();
		}

		friend	std::ostream& operator<<(std::ostream& os, const TTLDeque& data);

};

template <typename T>
std::ostream& operator<<(std::ostream& os, const TTLDeque<T>& data) {
	for (auto it = data._container.cbegin(); it != data._container.cend(); it++) {
		os << it->first() << ',' << it->second() << std::endl;
	}
}
