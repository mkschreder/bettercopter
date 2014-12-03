#pragma once

class LPFilter {
public:
	LPFilter(float cutoff){
		if(cutoff == 0) cutoff = 1.0; 
		mRC = 1.0/(cutoff*2*3.14);
    mPrevInput = 0; 
	}
	inline const float& output(){
		return mOutput; 
	}
	inline void update(float inp, float dt){
		float alpha = dt/(mRC+dt);
		float err = inp - mOutput; 
		mOutput += 0.5f * err;
	}
private:
	float mPrevInput, mRC, mOutput; 
};

class HPFilter {
public:
	HPFilter(float cutoff){
		if(cutoff == 0) cutoff = 1.0; 
		mRC = 1.0/(cutoff*2*3.14);
    mOutput = 0; 
	}
	inline const float& output(){
		return mOutput;
	}
	inline void update(float inp, float dt){
		float alpha = mRC/(mRC + dt); 
		float dInp = inp - mPrevInput; 
		mOutput = alpha * (mOutput + dInp);
		mPrevInput = inp; 
	}
private:
	float mRC, mOutput, mPrevInput; 
};

/// Low pass = Kp:alpha, Ki = 0, Kd = 0
/// 
class PIDFilter {
public:
	PIDFilter(float Kp, float Ki, float Kd){
		mKp = Kp;
		mKi = Ki;
		mKd = Kd;
	}
	inline float output(){
		return mOutput; 
	}
	inline void update(float inp){
		float err = inp - mOutput; 
		float dOut =
			(mKp * err) + // low pass pass
			(mKi * mAccError) +
			(mKd * (err - mPrevError)); // high pass
		mPrevError = err;
		mAccError += err;
		mOutput += dOut; 
	}
private:
	float mKp, mKi, mKd;
	float mPrevError;
	float mAccError;
	float mOutput; 
};

class PID {
public:
	PID(float Kp, float Ki, float Kd){
		mKp = Kp;
		mKi = Ki;
		mKd = Kd;
	}
	
	inline float output(){
		return mDOut; 
	}
	
	inline void update(float err){
		mDOut =
			(mKp * err) + // low pass pass
			(mKi * mAccError) +
			(mKd * (err - mPrevError)); // high pass
		mPrevError = err;
		mAccError += err;
	}
private:
	float mKp, mKi, mKd;
	float mPrevError;
	float mAccError;
	float mDOut; 
}; 
