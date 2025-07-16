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

# Satellite fix status information.
sensor_msgs/NavSatStatus status

# Latitude [decimal degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [decimal degrees]. Positive is east of prime meridian; negative is west.
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
