/***** signal.cpp *****/

#include "math.h"
#include "signal.h"

#define LN2 0.693

float ewma(float x,float s,float a)
{
	return (x * a + s * (1.f - a));	
}

float z_score(float mu, float sigma)
{
	if (sigma > 0.0f)
		return mu / sigma;
	else
		return 0.0f;
}


float lerp(float x, float a1, float b1, float a2, float b2)
{
	if (a1 == b1) {
		return a2;
	} else {
		return ((((b2 - a2) / (b1 - a1)) * (x - a1)) + a2);
	}
}


bool Schottky::update(float value)
{
	if (triggered) {
		triggered = (value > outThreshold);
		return false;
	} else if (value > inThreshold) {
		triggered = true;
		return true;
	} else {
		return false;
	}
}


bool TimedSchottky::update(float value)
{
	if (triggered) {
		triggered = ((value > outThreshold) || cc++ < d);
		return false;
	} else if (value > inThreshold) {
		cc = 0;
		triggered = true;
		return true;
	} else {
		return false;
	}
}


bool OneShot::active(uint64_t time)
{
	return (start <= time && time < start + duration);	
}


float OneShot::value(uint64_t time)
{
	return (active(time) ? amplitude : 0.0f);
}



bool Envelope::active(uint64_t time)
{
	return (start <= time && time < start + duration);	
}


float Envelope::value(uint64_t time)
{
	return (active(time) ? amplitude : 0.0f);
}


bool ADSR::active(uint64_t time)
{
	return (start <= time && time < start + duration + release);	
}


float ADSR::value(uint64_t time)
{
	if (active(time)) {
		uint64_t elapsed = time - start;
		float susAmp = amplitude * sustain;
		if (elapsed < attack)
		{
			return amplitude * (float(elapsed) / float(attack));
		} else if (elapsed < attack + decay) {
			float t = float(elapsed - attack) / float(decay);
			return t*susAmp + (1.0f - t)*amplitude;
		} else if (elapsed < duration) {
			return susAmp;
		} else if (elapsed < duration + release) {
			float t = float(elapsed - duration) / float(release);
			return (1.0f - t)*susAmp;			
		} else {
			return 0.0f;
		}

	} else {
		return 0.0f;
	}
}



bool ExpChirp::active(uint64_t time)
{
	return (start <= time && time < start + duration);	
}


float ExpChirp::value(uint64_t time)
{
	if (active(time)) {
		uint64_t elapsed = time - start;
		
		return baseValue * exp(double(elapsed)/(double(halfLife)*LN2));
		
		// float susAmp = amplitude * sustain;
		// if (elapsed < attack) {
		// 	return amplitude * (float(elapsed) / float(attack));
		// } else if (elapsed < attack + decay) {
		// 	float t = float(elapsed - attack) / float(decay);
		// 	return t*susAmp + (1.0f - t)*amplitude;
		// } else if (elapsed < duration) {
		// 	return susAmp;
		// } else if (elapsed < duration + release) {
		// 	float t = float(elapsed - duration) / float(release);
		// 	return (1.0f - t)*susAmp;			
		// } else {
		// 	return 0.0f;
		// }

	} else {
		return 0.0f;
	}
}




float SineWave::value(uint64_t t)
{
	if (t != last_time) {
		phase += double(freq) / 44100.0;
		phase = fmod(phase,1.0);
		last_time = t;
	}
	return amp * float(sin(2.0 * PI * phase));
}


float SineWave::value(uint64_t t, float inFreq, float inAmp)
{
	if (t != last_time) {
		phase += double(inFreq) / 44100.0;
		phase = fmod(phase,1.0);
		last_time = t;
	}

	return inAmp * float(sin(2.0 * PI * phase));
	//return float(phase);
}


float SawWave::value(uint64_t t)
{
	if (t != last_time) {
		phase += double(freq) / 44100.0;
		phase = fmod(phase,1.0);
		last_time = t;
	}
	return amp * float(phase);
}


float SawWave::value(uint64_t t, float inFreq, float inAmp)
{
	if (t != last_time) {
		phase += double(inFreq) / 44100.0;
		phase = fmod(phase,1.0);
		last_time = t;
	}

	return inAmp * float(phase);
	//return float(phase);
}

