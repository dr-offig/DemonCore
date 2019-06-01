/***** Brain.cpp *****/

#include "Brain.h"


//	int _port;
	// OSCClient _oscClient;
	// deque<oscpkt::Message> _outbox;
	// Transport _transport;
	// ShiftingPool<NoteOnset>_recordedHits;	

Brain::Brain() : _recordedHits(1024)
{

}

// Brain& operator=(const Brain& b)
// {
// 	if (this != &b) {
// 		_pClient = b._pClient;
// 		_bar = b._bar;
// 		_beat = b._beat;
// 		_tick = b._tick;
// 	}
// 	return *this;
// }


void Brain::setupOSC(int port)
{
	_oscClient.setup(port,"localhost");
}


void Brain::registerHit(int bar, int beat, int tick, int panel, int velocity)
{
	_recordedHits.add(NoteOnset(Transport(bar,beat,tick),panel,velocity));
	sendHit(bar+1,beat,tick,panel,velocity);
}


void Brain::registerBeat(int bar, int beat)
{
	_now.bar = bar;
	_now.beat = beat;
	_now.tick = 0;
	//printf("Bar: %d  Beat: %d\n", bar, beat);
	//sendHit(bar,beat,0,rand() % 5,metricStrength(beat));
	
}



void Brain::sendHit(int bar, int beat, int tick, int panel, int velocity)
{
	oscpkt::Message msg = _oscClient.newMessage.to("/schedule").add(bar).add(beat).add(tick).add(panel).add(velocity).end();
	_outbox.push_back(msg);
}


void Brain::think()
{
	while (!_outbox.empty()) {
		_oscClient.sendMessageNow(_outbox.front());
		_outbox.pop_front();
	}	
}


int Brain::metricStrength(int beat)
{
	if (beat % 4 == 0)
		return 100;
	else if (beat % 2 == 0)
		return 80;
	else
		return 60;
	
}