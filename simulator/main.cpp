#include "Application.h"
#include "time.h"

Application *App;

// Event receiver
class EventReceiverClass : public IEventReceiver  {

public:
	virtual bool OnEvent(const SEvent &TEvent)  {
		App->OnEvent(TEvent);
		return false; 
	}
};

int main() {
	App = new Application();
	
	// arm 
	while(1) {
		App->run(); 
	}

	return 0; 
}

