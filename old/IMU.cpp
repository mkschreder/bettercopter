#include "IMU.hpp"
#include <kernel.h>

IMU::IMU():
	mAccFilter({LPFilter(100), LPFilter(100), LPFilter(100)}),
	mGyroFilter({HPFilter(5), HPFilter(5), HPFilter(5)}),
	mMagFilter({LPFilter(10), LPFilter(10), LPFilter(10)})
{
}

quat IMU::getOrientation(){
	return mRotation; 
}

glm::mat3 IMU::getMatrix(){
	return mMatrix; 
}
	
vec3 IMU::getPosition(){
	return mPosition; 
}

vec3 IMU::getAngularVelocity(){

}

vec3 IMU::getLinearVelocity(){

}
	
void IMU::update(
		int16_t ax, int16_t ay, int16_t az, 
		int16_t gx, int16_t gy, int16_t gz, 
		int16_t mx, int16_t my, int16_t mz, 
		int16_t A, int16_t P, int16_t T, 
		float dt){
	
	if(dt == 0) dt = 0.01;
	
	mAcc = vec3(ax, ay, az) - mAccelOffset; 
	mGyro = vec3(gx, gy, gz);
	mMag = vec3(mx, my, mz);
		
	//int16_t heading = (int16_t)(atan2((double)myraw,(double)mxraw)*57.29578);

	// input into filters
	mAccFilter[0].update((float)ax, dt);
	mAccFilter[1].update((float)ay, dt); 
	mAccFilter[2].update((float)az, dt);

	mGyroFilter[0].update((float)gx, dt);
	mGyroFilter[1].update((float)gy, dt);
	mGyroFilter[2].update((float)gz, dt);

	mMagFilter[0].update((float)mx, dt);
	mMagFilter[1].update((float)my, dt);
	mMagFilter[2].update((float)mz, dt);
	
	mAcc = vec3(
		mAccFilter[0].output(), 
		mAccFilter[1].output(), 
		mAccFilter[2].output()
	);
	mGyro = vec3(
		mGyroFilter[0].output(), 
		mGyroFilter[1].output(), 
		mGyroFilter[2].output()
	);
	mMag = vec3(
		mMagFilter[0].output(), 
		mMagFilter[1].output(), 
		mMagFilter[2].output()
	);
	
	uart_printf(PSTR("A: [%d, %d, %d] \n"), 
		(int16_t)mAcc.x, 
		(int16_t)mAcc.y,
		(int16_t)mAcc.z);
	
	
	glm::vec3 mag = glm::vec3(mMag);
	mag -= glm::vec3(77.500000, 43.500000, -125.000000);
	mag = glm::normalize(-mag);
	
	glm::vec3 up = glm::normalize(mAcc);
	glm::vec3 gyr = mGyro;
	mag = glm::vec3(1.0f, 0.0f, 0.0f); 
	
	// center the sphere
	if(abs(glm::dot(up, mag)) < 0.99){
		glm::vec3 left = glm::normalize(glm::cross(mag, up));
		glm::vec3 front = glm::normalize(glm::cross(up, left));
		
		glm::mat3 mat = glm::mat3(glm::lookAt(
			glm::vec3(0.0f, 0.0f, 0.0f),
			glm::vec3(front.x, front.z, front.y), 
			glm::vec3(up.x, up.z, up.y)
		));
		/*
		glm::mat3 mat(
			front.x, up.y, left.x,
			front.y, up.x, left.y,
			front.z, up.z, left.z
		);*/
		//mMatrix = mat; 
		
		uart_printf(PSTR("MLEFT: %d %d %d "), 
			(int16_t)(mat[0].x * 100),
			(int16_t)(mat[0].y * 100),
			(int16_t)(mat[0].z * 100)); 
		uart_printf(PSTR("MFRONT: %d %d %d "), 
			(int16_t)(mat[1].x * 100),
			(int16_t)(mat[1].y * 100),
			(int16_t)(mat[1].z * 100)); 
		uart_printf(PSTR("MUP: %d %d %d \n"), 
			(int16_t)(mat[2].x * 100),
			(int16_t)(mat[2].y * 100),
			(int16_t)(mat[2].z * 100)); 
			
		/*glm::mat3 frame(
			1, -1, 0, 
			0, 1, 1, 
			1, 0, 0); */
			
		glm::quat q = 
			//glm::angleAxis(glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f)) * 
			//glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)) * 
			glm::quat(mat);
		
		// gyro compensation? 
		/*if(gyr.length() > 0){
			glm::quat grot = glm::angleAxis(glm::radians(gyr.length() * dt), gyr);
			//angle = 0.98 * (angle + gyro * dt) + 0.02 * (acc)
			mRotation = glm::slerp(glm::rotate(mRotation, glm::angle(grot), glm::axis(grot)), q, 0.5f);
			//mRotation = glm::slerp(mRotation * grot, q, 0.98f); 
			//glm::mat3 mat = glm::toMat3(grot) * mat; 
			//gh = glm::normalize(grot * gh);
		} else {
			mRotation = glm::slerp(mRotation, q, 0.4f);
		}*/
		mRotation = q; 
		//mRotation = glm::slerp(mRotation, q, 0.4f);
	} else {
		uart_printf(PSTR("NO VALID %d\n"), (int16_t)(abs(glm::dot(up, mag)) * 10000)); 
	}
}
