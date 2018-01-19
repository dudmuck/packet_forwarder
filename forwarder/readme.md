program # Class-B packet forwarder

This packet forwarder supports LoRaWAN class-B specification on gateway.   Class-B provides functionality of downlink messages to be received by low-power (mostly sleeping) end-nodes, without requiring prior uplink transmission by end-node.

Gateway does not implement any LoRaWAN protocol.  Gateway transfers packets between end-nodes and LoRaWAN server.  However, forwarder generates beacon packets because they provide timing reference from GPS receiver.

Requires json-c library, ie `apt-get install libjson-c-dev`

# Building forwarder
`lora_gateway` must be compiled as first step, HAL driver for gateway.  Both `lora_gateway` repository and `packet_forwarder` repository must reside in same directory.
1. `mkdir build`
2. `cd build`
3. `cmake ..`
4. `make`

# Configuration
Configuration is a two-step process in JSON format.  The hardware dependant portion is stored on gateway with this forwarder software application.  The region-specific portion is provided from LoRaWAN server, defining the receive frequencies of the gateway.

Upon forwarder startup, the hardware-specific portion is first loaded, then after receiving remaining configuration from server, the concentrator will be started.  Hardware specific portion contains `txlut`, which radio is used for transmit, RSSI offsets, radio clock source, and LBT enable.  Regional configuration portion contains LBT frequencies, center frequency of each radio, and intermediate frequency offset of each receive channel. 

`gateway_conf` section of configuration defines GPS serial device to be used, LoRaWAN server name and port, polling rate of SX1301 receiver and server connection retry interval.

If server has changed region assignment of gateway, server will close TCP connection to gateway, causing the forwarder to stop concentrator and reload configuration before restarting.

Gateway EUI is used to uniquely identify gateway to LoRaWAN server.  Gateway EUI is read from MAC address of gateway host.  In function `get_host_unique_id()` an ioctl is performed to find network interface, and another ioctl to retrieve MAC address of that interface.

# Beacon generation
Class-B beacon is transmitted autonomously by this packet forwarder, but the frequency, bandwidth and datarate of beacon is defined by server region assignment in `lorawan` section of JSON configuration.

To keep server synchronized with each gateway, every time forwarder transmit a beacon, an indication packet is sent to server.  This beacon indication contains GPS time which was transmitted over the air, along with the SX1301 counter value captured at the PPS pulse which triggered beacon transmission.  Beacons are transmitting using SX1301 `ON_GPS` method because this packet used as a timing reference for motes.

On server, the GPS time of beacon is used to calculate ping slot offset for each mote, and SX1301 counter value at beacon is used as reference for transmitting downlink packets sent on ping slots.  Ping slot downlinks are set using SX1301 `TIMESTAMPED` mode.  SX1301 counter value at beacon is also used by server to generate `BeaconTimingAns`, informing mote when next beacon occurs.

# Connection to server
TCP connection is used to server. Server name and port is defined in JSON configuration file.  Forwarder autonomously attempts to reconnect if not connected to server.

# Standard input
`STDIN` is read for user input by forwarder.  It can be access when program started from terminal.  User can query status from terminal and perform lab testing, such as temporarily skippping beacon transmission or downlink transmission.

# Forwarder operation
Threads are not used.  Forwarder runs as single process. `select()` is used to multiplex I/O with connection to server, GPS serial port and `STDIN`.  `select()` is run with timeout to permit polling of SX1301 receiver.
### downlink transmit queue
Simple circular buffer is used to hold gateway transmit requests from server.  Downlink transmit packets are sent out of order by server because join accept message is sent with longer delay than other downlink packets.  All downlink packets are sent using SX1301 `TIMESTAMPED` mode.
