/***** Transport.h *****/

#ifndef __TRANSPORT__
#define __TRANSPORT__

class Transport
{
public:
	Transport() : bar(0), beat(0), tick(0) {}
	Transport(int aBar, int aBeat, int aTick) : bar(aBar), beat(aBeat), tick(aTick) {}
	
	void update(int aBar, int aBeat, int aTick) { bar = aBar; beat = aBeat; tick = aTick; }
	int bar;
	int beat;
	int tick;
	
};


class NoteOnset
{
public:
	NoteOnset() : bbt(), midiPitch(0), midiVel(0) {}
	NoteOnset(Transport tp, int pitch, int vel) : bbt(tp), midiPitch(pitch), midiVel(vel) {}
	
	Transport bbt;
	int midiPitch;
	int midiVel;
	
};

#endif