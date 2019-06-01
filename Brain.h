/***** Brain.h *****/

#ifndef __BRAIN__
#define __BRAIN__

#include <Bela.h>
#include <OSCClient.h>
#include <vector>
#include "Tonality.h"
#include "Transport.h"
#include "RingBuffer.h"

#define MAX_RECORDED_HITS 1024

class Brain
{
public:
	Brain();
	~Brain() {}
	//Brain(const Brain& b);
	//Brain& operator=(const Brain&);    

	void think();
	void setupOSC(int port);
	void registerHit(int bar, int beat, int tick, int panel, int velocity);	
	void registerBeat(int bar, int beat);
	void sendHit(int bar, int beat, int tick, int panel, int velocity);

	int metricStrength(int beat);
	
		
private:
	OSCClient _oscClient;
	deque<oscpkt::Message> _outbox;
	Transport _now;
	ShiftingPool<NoteOnset>_recordedHits;	
};





#endif