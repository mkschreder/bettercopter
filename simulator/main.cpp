//***************************************************************
// Bullet/irrlicht demo by Alan Witkowski
// http://www.cs.utah.edu/~witkowsk
// http://code.google.com/p/irrlamb/
//***************************************************************

#include "Application.h"
#include "time.h"
#include "kernel.h"

Application *App;

// Event receiver
class EventReceiverClass : public IEventReceiver  {

public:
	virtual bool OnEvent(const SEvent &TEvent)  {
		App->OnEvent(TEvent);
	}
};

extern "C" void app_init(void); 
extern "C" void app_loop(void); 

void sim_loop(); 
void sim_init() {
	App = new Application();
	
	// arm 
	timeout_t tim = timeout_from_now(1200000); 
	set_pin(RC_THROTTLE, 1000); 
	set_pin(RC_ROLL, 1900); 
	while(!timeout_expired(tim))app_loop(); 
	set_pin(RC_ROLL, 1500); 
	set_pin(RC_YAW, 1500); 
	set_pin(RC_PITCH, 1500); 
}

void sim_loop() {
	App->run(); 
}
/*
int sim_main() {
	App = new Application();
	
	app_init(); 
	
	// arm 
	timeout_t tim = timeout_from_now(1200000); 
	set_pin(RC_THROTTLE, 1000); 
	set_pin(RC_ROLL, 1900); 
	while(!timeout_expired(tim))app_loop(); 
	set_pin(RC_ROLL, 1500); 
	set_pin(RC_YAW, 1500); 
	set_pin(RC_PITCH, 1500); 
	
	while(1) {
		app_loop(); 
		App->run(); 
	}
	return 0;
}*/
