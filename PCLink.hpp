#pragma once

/**
PCLink uses a JSON based protocol and parses the incoming data once 
there is something to parse. It uses a set of primitive commands to 
set and get configuration values. 

To set a configuration variable, this is set to the PC link. As a 
result of this, it will call OnSetValue(name, value) of the listener: 
{"set": "some_name", "value": "some_value"}

To get a value, use this: 
{"get": "some_name"}

Values are represented as strings, and should be converted by 
application code. 
*/

#include "mavlink.h"
#include "types.hpp"

class PCLink {
public:
	PCLink(); 
	
	bool ReceiveMessage(mavlink_message_t *msg); 
	void SendHeartbeat(copter_state_t state); 
	void SendRawIMU(uint64_t usec, 
		const glm::vec3 &acc, const glm::vec3 &gyr, const glm::vec3 &mag); 
	void SendAttitude(uint32_t timestamp, 
		float yaw, float pitch, float roll, 
		float rate_yaw, float rate_pitch, float rate_roll); 
	void SendHud(
		float airspeed, float groundspeed, int16_t heading, 
		uint16_t throttle, float alt, float climb);
	void SendParamValueFloat(const char *name, float val, int count, int index); 
	
	void SetSerialInterface(serial_dev_t serial){mSerial = serial;}
private: 
	serial_dev_t mSerial; 
}; 
