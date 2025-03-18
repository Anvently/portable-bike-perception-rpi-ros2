import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix
from cyclosafe.src.ASerialSensor import ASerialPublisher

"""
$GNGLL,4850.45401,N,00235.23105,E,111859.00,A,A*7E                              GPS+GLONASS, geographic position, latitude/longitude
$GNRMC,111900.00,A,4850.45398,N,00235.23110,E,0.139,,180325,,,A*6F              | recommended minimum specific GPS data
$GNVTG,,T,,M,0.139,N,0.258,K,A*39                                               | course over ground and ground speed
$GNGGA,111900.00,4850.45398,N,00235.23110,E,1,10,1.20,88.2,M,46.2,M,,*75        | gps fixed data
$GNGSA,A,3,18,31,28,25,04,16,,,,,,,2.14,1.20,1.77*1C                            | gnss dop and active satellite
$GNGSA,A,3,86,75,76,67,,,,,,,,,2.14,1.20,1.77*15                                | gnss dop and active satellite
$GPGSV,3,1,12,04,11,317,12,05,15,078,,09,02,345,15,16,20,293,24*72              GPS gnss satellites in view
$GPGSV,3,2,12,18,44,163,29,20,13,043,16,25,32,114,24,26,47,296,20*71            | 
$GPGSV,3,3,12,28,50,212,19,29,64,059,,31,57,255,19,49,34,177,*78                |
$GLGSV,3,1,10,66,00,281,,67,11,328,19,68,07,014,,74,13,133,*61                  GLONASS
$GLGSV,3,2,10,75,65,125,30,76,48,319,30,77,04,316,,84,24,034,*69                |
$GLGSV,3,3,10,85,76,068,14,86,38,200,32*65                                      |

GLL => geographic position, latitude/longitude
RMC => recommended minimum specific GPS data
VTG => course over ground and ground speed
GGA => gps fixed data
GSA => gnss dop and active satellites
GSV => gnss satellites in view
Ex: 
 3,1,10,66,00,281,,67,11,328,19,68,07,014,,74,13,133,*61       
 3, 1 => message 1/3
 10 => total nbr of satellite in view
 66 => PRV of satellite (ID)
 00, 281, ?, 67 => élévation, azimut, SNR

- on veut :
	- latitude/longitude => GLL
	- hdop => GSA
	

"""


class GPSPublisher(ASerialPublisher):
	
	def __init__(self):
		super().__init__(NavSatFix, 'gps', '/dev/ttyACM0', 115200)
		self.latitude = None
		self.longitude = None
		self.hdop = 5

	def parse(self) -> bool:
		"""Update stored values from GPS input. Return true if one or more value were updated"""
		update: bool = False
		lines = self.buffer.split(b'\n')
		for line in lines:
			words = line.decode("utf-8").split(',')
			message_type = words[0]
			if (message_type == "$GNGLL"):
				self.latitude = float(words[1])
				self.longitude = float(words[3])
				update = True
			elif (message_type == "$GNGSA"):
				self.hdop = float(words[16])
				update = True
		if len(lines) > 1:
			self.buffer = b'\n'.join(lines[:-1])
		print(hey)
		return update
	
	def publish(self, data: NavSatFix):
		msg = NavSatFix(latitude=self.latitude, longitude = self.longitude)
		msg.position_covariance[0] = self.hdop ** 2
		msg.position_covariance[4] = self.hdop ** 2
		msg.position_covariance[8] = (2 * self.hdop) ** 2
		msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
		self.get_logger().info(f"lat={msg.latitude}, lon={msg.longitude}, hdop={self.hdop}")
		self.pub.publish(msg)

def main(args=None):
	try:
		rclpy.init(args=args)
		sensor_publisher = GPSPublisher()
		rclpy.spin(sensor_publisher)
	
	except (KeyboardInterrupt, ExternalShutdownException):
		pass


if __name__ == '__main__':
	main()