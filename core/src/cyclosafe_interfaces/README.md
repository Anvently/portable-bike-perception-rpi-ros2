# cyclosafe_interfaces

Définit les interfaces ROS 2 propres au projet Cyclosafe, à savoir :
- ***cyclosafe_interfaces/msg/NavSatinfo***
- ***cyclosafe_interfaces/srv/SaveImages***

# msg/NavSatInfo

Reprend le contenu de `sensor_msgs/msg/NavSatFix` en y intégrant des données supplémentaires comme l'altitude, la vitesse, le nombre de satellites utilisées et la dop (hdop et vdop).

~~~
$ ros2 interface show cyclosafe_interfaces/msg/NavSatInfo
~~~
<ins>**Résultat</ins>** :
~~~
# Navigation Satellite fix for any Global Navigation Satellite System
#
# Specified using the WGS 84 reference ellipsoid

# header.stamp specifies the ROS time for this measurement (the
#        corresponding satellite time may be reported using the
#        sensor_msgs/TimeReference message).
#
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# Satellite fix status information.
sensor_msgs/NavSatStatus status
	#
	int8 STATUS_UNKNOWN = -2        #
	int8 STATUS_NO_FIX =  -1        #
	int8 STATUS_FIX =      0        #
	int8 STATUS_SBAS_FIX = 1        #
	int8 STATUS_GBAS_FIX = 2        #
	int8 status -2 #
	uint16 SERVICE_UNKNOWN = 0  #
	uint16 SERVICE_GPS =     1
	uint16 SERVICE_GLONASS = 2
	uint16 SERVICE_COMPASS = 4      #
	uint16 SERVICE_GALILEO = 8
	uint16 service

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Horizontal dilution of precision
float32 hdop

# Position dilution of precision
float32 pdop

# Ground speed in kphrs
float32 ground_speed

# Number of active satellites
int32 actives_sat
~~~


# srv/SaveImages

Service permettant l'interaction entre un précédent noeud hub (devenu obsolete avec l'utilisation de ROS bag) et le [**noeud caméra**](../cyclosafe/README.md#acamera). Demande au noeud caméra d'enregistrer les images prises sur les X dernières millisecondes (`time`) au `path` .demandé

 ~~~
 $ ros2 interface show cyclosafe_interfaces/srv/SaveImages
 ~~~~

<ins>**Résultat</ins>** :

 ~~~
 uint32 time # milliseconds
string path # folder where to save images
---

uint8 SUCCESS = 0 # time constraint were respected
uint8 PARTIAL_SUCCESS = 1 # time constraint partially respected (some images were missing)
uint8 FAILURE = 2 # an error occured or invalid argument

uint8 result
~~~

Voir le [**noeud caméra**](../cyclosafe/README.md#acamera) pour un exemple d'utilisation.
