#include <deque>
#include <chrono>
#include <algorithm>
#include <ostream>

/*
TTL based container.
Store pairs of time and any data type 
ttl check is done at each add

*/
std::ostream&	operator<<(std::ostream& os, const sensor_msgs::msg::Range& msg);
std::ostream&	operator<<(std::ostream& os, const sensor_msgs::msg::NavSatFix& msg);

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
			return (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - _start));
		}
		
	public:
	
		TTLDeque(std::chrono::milliseconds ttl, TimePoint start) : _ttl(ttl), _start(start) {
			std::cout << "ttl=" << _time().count() << std::endl;
		}

		TTLDeque(std::chrono::milliseconds ttl) : _ttl(ttl) {
			_start = std::chrono::system_clock::now();
			
		}

		void	cleanup() {
			std::chrono::milliseconds	expires_before = _time() - _ttl;
	
			auto upper = std::upper_bound(_container.begin(), _container.end(), expires_before,
				[](const std::chrono::milliseconds& t, const Item& item) {
					return t < item.first;
				}
			);
			if (upper == _container.end())
				return;
			_container.erase(_container.begin(), upper);
		}
		
		void	push(const TimePoint& at,  const T& item) {
			std::cout << "Adding date=" << std::format("{0:%Y%m%d-%H%M}", at) << std::endl;
			_container.emplace_back(std::chrono::duration_cast<std::chrono::milliseconds>(at - _start), item);
		}

		void	clear() {
			_container.clear();
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

		friend	std::ostream& operator<<(std::ostream& os, const TTLDeque<T>& data) {
			for (typename std::deque<Item>::const_iterator it = data._container.cbegin(); it != data._container.cend(); it++) {
				os << std::to_string(it->first.count()) << ',' << it->second << std::endl;
			}
			return (os);
		}

};

// template <typename T>
// std::ostream& operator<<(std::ostream& os, const TTLDeque<T>& data) {
// 	for (auto it = data._container.cbegin(); it != data._container.cend(); it++) {
// 		os << std::to_string(it->first().count()) << ',' << it->second() << std::endl;
// 	}
// 	return (os);
// }
