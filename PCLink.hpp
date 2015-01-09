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

#define PCLINK_BUFFER_SIZE 128

class PCLinkListener {
	friend class PCLink; 
	virtual void OnPCLinkSetValue(const char *name, const char *value) = 0; 
	virtual int OnPCLinkGetValue(const char *name, char *buffer, uint16_t size) = 0; 
}; 

class PCLink {
public:
	PCLink(PCLinkListener *ls); 
	
	void ProcessEvents(); 
	
	void SetSerialInterface(serial_dev_t serial){mSerial = serial;}
private: 
	char 	buffer[PCLINK_BUFFER_SIZE]; // buffer for holding string 
	uint16_t 	buffer_ptr; 
	
	PCLinkListener *mListener; 
	serial_dev_t mSerial; 
	uint8_t mError; 
}; 
