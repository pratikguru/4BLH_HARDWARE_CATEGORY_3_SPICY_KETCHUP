# Asset-Tracking
A simple Asset Tracking system using WiFi nodes from ESP32 and Magnetometry.


The basic functionality is to retrieve a vector of distance and signal strength and translate that vector to pixel mapping. Most wireless system's have a functionality to get the signal strength in the form of RSSI, the RSSI value is linear proportional to the distance of the signal source. 

This mapping system can be used to find the proximity between two nodes, one drawback for tracking with such an architecture is the fact that distance is scalar and vector is more useful since it also produces absolute positioning.

The direction of the signal can be found using two techniques.
* Angle of Attack.
* Magnetrometry.
* Trilateration.
* Multi lateration. 

### Angle of Attack (AoA)
This method can turn out to be very expensive as this requires an array of antenna's placed along an axis.

### Magnetometry
This method is comparatively cheap but requires complex algorithms and calibration.

### Trilateration and Multi lateraion
This method is far more accurate than the other too but is very expensive and complex to develop. This solution can get very complicated when there are multiple nodes that have to be scanned.
