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
	void SendHeartbeat(); 
	void SendRawIMU(uint64_t usec, 
		const glm::vec3 &acc, const glm::vec3 &gyr, const glm::vec3 &mag); 
	void SendAttitude(uint32_t timestamp, 
		float roll, float pitch, float yaw, 
		float rate_roll, float rate_pitch, float rate_yaw); 
	
	void SetSerialInterface(serial_dev_t serial){mSerial = serial;}
private: 
	serial_dev_t mSerial; 
}; 
