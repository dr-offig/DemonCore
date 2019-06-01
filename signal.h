/***** signal.h *****/

#ifndef _DODECAHEDRUM_SIGNAL_
#define _DODECAHEDRUM_SIGNAL_

#include <Bela.h>

#define PI 3.1415926535
#define TWOPI 6.28318531
typedef uint64_t SID;


class UniqueID
{
public:
	UniqueID(SID offset) : lastID(offset) {}
	SID generate() { return ++lastID; }
		
private:
	uint64_t lastID;
};


float ewma(float x, float s, float a);
float z_score(float mu, float sigma);
float lerp(float x, float a1, float a2, float b1, float b2);


class Schottky
{
public:
	Schottky(float inThresh) : inThreshold(inThresh), outThreshold(0.5f * inThresh), triggered(false) {}
	Schottky(float inThresh, float outThresh) : inThreshold(inThresh), outThreshold(outThresh), triggered(false) {}
		
	virtual bool update(float value);

	float inThreshold;
	float outThreshold;

protected:
	bool triggered;
	
};


class TimedSchottky : Schottky
{
public:
	TimedSchottky(float inThresh, float outThresh, unsigned int suppressCount) : Schottky(inThresh, outThresh), d(suppressCount), cc(0) {}
	
	virtual bool update(float value);
	
	unsigned int d;

private:
	unsigned int cc;
	
};


class OneShot
{
public:
	// Constructors
	OneShot() : start(0), duration(512), amplitude(1.0f), trigger(false) {}
	OneShot(uint64_t strt, uint64_t dur, float amp) : start(strt), duration(dur), amplitude(amp), trigger(false) {}
	
	// member functions
	bool active(uint64_t time);
	float value(uint64_t time);
	
	// member variables
	uint64_t start;
	uint64_t duration;
	float amplitude;
	bool trigger;

};


class Envelope
{
public:
	// Constructors
	Envelope() : start(0), duration(44100), amplitude(1.0f), trigger(false), id(0), carrier_id(0) {}
	virtual ~Envelope() {}
	Envelope(uint64_t strt) : start(strt), duration(44100), amplitude(1.0f), trigger(false), id(0), carrier_id(0) {}
	Envelope(uint64_t strt, uint64_t dur, float amp) : start(strt), duration(dur), amplitude(amp), trigger(false) {}
	// Copy constructor 
    Envelope(const Envelope& e) {start = e.start; duration = e.duration; amplitude = e.amplitude; trigger = e.trigger; id=e.id; carrier_id=e.carrier_id; } 
	
	// member functions
	virtual bool active(uint64_t time);
	virtual float value(uint64_t time);
	
	// member variables
	uint64_t start;
	uint64_t duration;
	float amplitude;
	bool trigger;
	SID id;
	SID carrier_id;
};


class ADSR : public Envelope
{
	public:
	ADSR() : Envelope(0,44100,1.0f),attack(441),decay(441),sustain(0.8f),release(1000){}
	ADSR(uint64_t strt) : Envelope(strt,44100,1.0f),attack(441),decay(441),sustain(0.8f),release(1000){}
	ADSR(uint64_t strt,uint64_t dur,float amp,
				uint64_t a,uint64_t d,float s,uint64_t r) 
			: Envelope(strt,dur,amp),attack(a),decay(d),sustain(s),release(r) {}
	virtual ~ADSR() {}
	ADSR(const ADSR& a) : Envelope(a) { attack = a.attack; decay = a.decay; sustain = a.sustain; release = a.release; }
	// member functions
	virtual bool active(uint64_t time);
	virtual float value(uint64_t time);

	uint64_t attack;
	uint64_t decay;
	float sustain;
	uint64_t release;
	
};


class ExpChirp : public Envelope
{
	public:
	ExpChirp() : Envelope(0,44100,1.0f), baseValue(100.0f), halfLife(10000), direction(1) {}
	ExpChirp(uint64_t strt) : Envelope(strt,44100,1.0f), baseValue(100.0f), halfLife(10000), direction(1) {}
	ExpChirp(uint64_t strt,uint64_t dur,float amp,
				float bv, uint64_t hl,uint64_t d) 
			: Envelope(strt,dur,amp), baseValue(bv), halfLife(hl), direction(d) {}
	virtual ~ExpChirp() {}
	ExpChirp(const ExpChirp& a) : Envelope(a) { baseValue = a.baseValue; halfLife = a.halfLife; direction = a.direction; }
	// member functions
	virtual bool active(uint64_t time);
	virtual float value(uint64_t time);

	float baseValue;
	uint64_t halfLife;
	uint64_t direction;
};



class Carrier
{
public:
	Carrier() : id(0), freq(1000.0f), amp(1.0f), phase(0.0), start_time(0), last_time(0) {}
	Carrier(SID some_id) : id(some_id), freq(1000.0f), amp(1.0f), phase(0.0), start_time(0),last_time(0) {}
	Carrier(SID some_id, float some_freq, float some_amp) : id(some_id), freq(some_freq), amp(some_amp), phase(0.0), start_time(0),last_time(0) {}
	virtual ~Carrier() {}
	Carrier(const Carrier& c) { id = c.id; freq = c.freq; amp = c.amp; phase = c.phase; start_time = c.start_time; last_time = c.last_time; }
	//virtual float value(uint64_t time,float freq,float amp) = 0;
	virtual float value(uint64_t t) = 0;
	virtual float value(uint64_t t,float inFreq, float inAmp) = 0;
	
	SID id;
	float freq;
	float amp;
	
protected:
	double phase;
	uint64_t start_time;
	uint64_t last_time;	
	
};


class SineWave : public Carrier
{
public:
	SineWave() : Carrier() {}
	SineWave(SID some_id) : Carrier(some_id) {}
	SineWave(SID some_id, float some_freq, float some_amp) : Carrier(some_id, some_freq, some_amp) {}
	virtual ~SineWave() {}
	SineWave(const SineWave& sw) : Carrier(sw) {}
	
	virtual float value(uint64_t t);
	virtual float value(uint64_t t,float inFreq, float inAmp);
};



class SawWave : public Carrier
{
public:
	SawWave() : Carrier() {}
	SawWave(SID some_id) : Carrier(some_id) {}
	SawWave(SID some_id, float some_freq, float some_amp) : Carrier(some_id, some_freq, some_amp) {}
	virtual ~SawWave() {}
	SawWave(const SawWave& sw) : Carrier(sw) {}
	
	virtual float value(uint64_t t);
	virtual float value(uint64_t t,float inFreq, float inAmp);
};


#endif
