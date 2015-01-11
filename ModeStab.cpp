#include "ModeStab.hpp"
#include <kernel.h>
#include "mavlink.h"

// defualts

#define STAB_PID_KP 4.5 // change 0 - 50
#define STAB_PID_KI 0
#define STAB_PID_KD 0
#define STAB_PID_MAX 30.0

#define RATE_PID_KP 0.5 // change 
#define RATE_PID_KI 0.05
#define RATE_PID_KD 0.04
#define RATE_PID_MAX 500.0

ModeStab::ModeStab() : 
	mPID({
		AC_PID(STAB_PID_KP, STAB_PID_KI, STAB_PID_KD, STAB_PID_MAX), // pitch
		AC_PID(10, 0, 10, 30), // yaw
		AC_PID(STAB_PID_KP, STAB_PID_KI, STAB_PID_KD, STAB_PID_MAX), // roll
		AC_PID(RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_MAX), 
		AC_PID(RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_MAX), 
		AC_PID(RATE_PID_KP, RATE_PID_KI, RATE_PID_KD, RATE_PID_MAX),
		//AC_PID(ALT_PID_KP, 	ALT_PID_KI,  ALT_PID_KD, 	ALT_PID_MAX)
	}){
	mAccPitch = mAccYaw = mAccRoll = 0; 
}

void ModeStab::Reset(){
	for(int c = 0; c < STAB_PID_COUNT; c++){
		mPID[c].reset_I(); 
	}
}

ThrottleValues ModeStab::ComputeThrottle(float dt, const RCValues &rc, const glm::vec3 &raw_gravity, const glm::vec3 &raw_omega){
	// get normalized gravity vector
	glm::vec3 nacc = glm::normalize(raw_gravity);
	//float pp = 0, py = 0, pr = 0; 
	float ap = 0, ay = 0, ar = 0; 
	
	ap = glm::degrees(::atan2(nacc.y , nacc.z )); 
	ay = 0; 
	ar = glm::degrees(::atan2(nacc.x , nacc.z )); 
	
	float gp, gy, gr; 
	gp = raw_omega.x * dt; //0.9 * gp + gyr.x * 0.1; 
	gy = raw_omega.z * dt; //0.9 * gy + gyr.y * 0.1; 
	gr = raw_omega.y * dt; //0.9 * gr + gyr.z * 0.1; 
	
	float rcp = -map(rc.pitch, 1000, 2000, -25, 25); //(pitch - 1500.0); 
	float rcr = -map(rc.roll, 1000, 2000, -25, 25); //(roll - 1500.0); 
	float rcy = -map(rc.yaw, 1000, 2000, -50, 50); //(yaw - 1500.0); 
	
	//fuse accelerometer and gyroscope into ypr using comp filter
	mAccPitch = 0.98 * (mAccPitch + gp) + 0.02 * ap; // needs to be + here for our conf front/back/left/right
	//mAccYaw 	= 0.98 * (mAccYaw 	- gy) + ; 
	// integrate gyro directly, but filter out low level noise
	mAccYaw 	+= (abs(raw_omega.z) > 2)?(gy):0; 
	mAccRoll 	= 0.98 * (mAccRoll 	- gr) + 0.02 * ar; 
	
	// control target yaw using control sticks
	mTargetYaw = mTargetYaw + rcy * dt; 
	
	// calculate desired rotation rate in degrees / sec
	float sp = constrain(mPID[PID_STAB_PITCH].get_pid(rcp - mAccPitch, dt), -250, 250); 
	float sr = constrain(mPID[PID_STAB_ROLL].get_pid(rcr 	- mAccRoll, dt), -250, 250); 
	float sy = 0; //constrain(mPID[PID_STAB_YAW].get_pid(gy, dt), -360, 360); 
	
	// calculate the actual rate based on current gyro rate
	float rp = constrain(mPID[PID_RATE_PITCH].get_pid(sp - gp, dt), -500, 500); 
	float rr = constrain(mPID[PID_RATE_ROLL].get_pid(sr - gr, dt), -500, 500); 
	float ry = constrain(mPID[PID_RATE_YAW].get_pid(sy - gy, dt), -500, 500); 
	
	{
		mavlink_attitude_t attmsg; 
		static uint32_t time_boot_ms = 0; time_boot_ms++; 
		attmsg.time_boot_ms = time_boot_ms; 
		attmsg.roll = glm::radians(mAccRoll); ///< Roll angle (rad, -pi..+pi)
		attmsg.pitch = glm::radians(mAccPitch); ///< Pitch angle (rad, -pi..+pi)
		attmsg.yaw = glm::radians(mAccYaw); ///< Yaw angle (rad, -pi..+pi)
		attmsg.rollspeed = glm::radians(raw_omega.x); ///< Roll angular speed (rad/s)
		attmsg.pitchspeed = glm::radians(raw_omega.z); ///< Pitch angular speed (rad/s)
		attmsg.yawspeed = glm::radians(raw_omega.y); ///< Yaw angular speed
		
		mavlink_system_t mavlink_system;
		mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
		mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
		
		// Initialize the required buffers
		mavlink_message_t msg;
		uint8_t buf[100];
		 
		// Pack the message
		mavlink_msg_attitude_encode_chan(20, MAV_COMP_ID_IMU, 0, 
			&msg, &attmsg); 
		
		uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
		
		uart0_putn((char*)buf, len);
	}
	
	frame_log("\"conv_rc_pitch\": %-4d, \"conv_rc_roll\": %-4d, \"conv_rc_yaw\": %-4d, ", 
		(int16_t)(rcp ), (int16_t)(rcr ), (int16_t)(rcy )); 
	frame_log("\"acc_pitch\": %-4d, \"acc_roll\": %-4d, \"acc_yaw\": %-4d,",
		(int16_t)(mAccPitch * 100), (int16_t)(mAccRoll * 100), 
		(int16_t)(mAccYaw * 100)); 
	frame_log("\"target_yaw\": %-4d, ", (int16_t)(mTargetYaw * 100)); 
	frame_log("\"stab_pitch\": %-4d, \"stab_roll\": %-4d, \"stab_yaw\": %-4d, ", 
		(int16_t)(sp), (int16_t)(sr), (int16_t)(sy)); 
	frame_log("\"rate_pitch\": %-4d, \"rate_roll\": %-4d, \"rate_yaw\": %-4d, ", 
		(int16_t)(rp), (int16_t)(rr), (int16_t)(ry)); 
	
	return glm::i16vec4(
		// front
					+ rp + ry,
		// back  
					- rp + ry,
		// left 
		- rr			 - ry,
		// right
		+ rr			 - ry
	); 
}
