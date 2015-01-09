#include <stddef.h>
#include <kernel.h>
#include <util/json.h>
#include <stdlib.h>
#include <string.h>
#include "PCLink.hpp"

PCLink::PCLink(PCLinkListener *l){
	mListener = l; 
	mError = 0; 
	mSerial = 0; 
	buffer_ptr = 0; 
}

/// Basically: get characters until we either have a full buffer or a new line
/// then parse the json and call callbacks. If new line is reached, set error
/// and continue, but when next new line is reached, if error set then 
/// reset the error and start new capture
void PCLink::ProcessEvents(){
	if(!mSerial) return; 
	
	uint16_t ch = serial_getc(mSerial); 
	if(ch == SERIAL_NO_DATA) return; 
	ch = ch & 0xff; 
	
	serial_putc(mSerial, ch); 
	
	if(ch == '\r'){
		kprintf("PARSE\n"); 
		if(mError){
			mError = 0; // start new capture
			kprintf("NEW\n"); 
			return; 
		}
		// try to parse the data
		json_parser parser; 
		jsontok_t tokens[10]; 
		memset(tokens, 0, sizeof(tokens)); 
		json_init(&parser); 
		int tok_count = json_parse(&parser, buffer, buffer_ptr, tokens, 10); 
		if(tok_count > 2 && tokens[0].type == JSON_OBJECT){
			char act[16]; 
			memset(act, 0, sizeof(act)); 
			int count = tokens[1].end - tokens[1].start; 
			if(count > 15) count = 15; 
			memcpy(act, &buffer[tokens[1].start], tok_count); 
			if(pgm_streq(act, PSTR("set")) && count == 4){
				kprintf("SET"); _delay_ms(100); 
				char name[16]; 
				char value[16]; 
				uint16_t count = tokens[1].end - tokens[1].start; 
				if(count > sizeof(name)-1) count = sizeof(name) - 1; 	
				strncpy(name, &buffer[tokens[1].start], count); 
				count = tokens[3].end - tokens[3].start; 
				if(count > sizeof(name)-1) count = sizeof(name) - 1; 	
				strncpy(name, &buffer[tokens[3].start], count); 
				mListener->OnPCLinkSetValue(name, value); 
			} else if(pgm_streq(act, PSTR("get"))){
				char name[16]; 
				char value[16]; 
				char str[64]; 
				uint16_t count = tokens[1].end - tokens[1].start; 
				if(count > sizeof(name)-1) count = sizeof(name) - 1; 	
				strncpy(name, &buffer[tokens[1].start], count); 
				
				kprintf("GET"); _delay_ms(100); 
				if(mListener->OnPCLinkGetValue(name, value, sizeof(value))){
					
				kprintf("GET2"); _delay_ms(100); 
					snprintf(str, sizeof(str), "{\"var\":\"%s\",\"val\":\"%s\"}\n", name, value); 
					serial_putn(mSerial, (uint8_t*)str, strlen(str)); 
				} 
			} else {
				kprintf("INVACT:%s\n", act); 
			}
		} else {
			kprintf("NOPARSE:%d,%s,%d\n", tok_count, buffer, buffer_ptr); 
		}
		buffer_ptr = 0; 
	} else if(mError){
		kprintf("ERROR\n"); 
		return; 
	} else {
		// put the character into buffer 
		buffer[buffer_ptr++] = ch; 
		buffer[buffer_ptr] = 0; 
		if(buffer_ptr == (PCLINK_BUFFER_SIZE - 1)){
			mError = 1; 
			buffer_ptr = 0; 
		}
	}
}
