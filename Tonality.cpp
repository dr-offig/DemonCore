/***** Tonality.cpp *****/

#include "Tonality.h"
#include <stdlib.h>

unsigned modulo(int x, int b)
{
	int c = x % b;
	if (c < 0)
		c+= abs(b);
	
	return(unsigned(c));		
}


unsigned mod12(int x)
{
	int c = x % 12;
	if(c < 0)
		c+= 12;
	
	return(unsigned(c));		
}


int PitchClass::pitchInOctave(int octave)
{
	return doremi + 12 * octave;
}


Scale::Scale(PitchClass& tnc, int* stepHeights, unsigned numSteps) : _tonic(tnc)
{
	PitchClass pc = tnc;
	for (unsigned j=0; j<numSteps; j++) {
		_steps.push_back(stepHeights[j]);
		_degrees.push_back(pc);	
		pc = pc + stepHeights[j];	
	}		
}


void Scale::transposeTo(PitchClass& newTonic)
{
	int interval = newTonic - _tonic;
	_tonic = newTonic;
	unsigned J = _degrees.size();
	for (unsigned j = 0; j < J; j++) {
		_degrees[j] = _degrees[j] + interval;
	}	
	
}


PitchClass Scale::transposeBy(int& interval)
{
	_tonic = _tonic + interval;
	unsigned J = _degrees.size();
	for (unsigned j = 0; j < J; j++) {
		_degrees[j] = _degrees[j] + interval;
	}	
	return _tonic;
}




int Scale::quantiseUp(int pitch)
{
	unsigned J = _degrees.size();
	for (int interval=0; interval<12; interval++) {
		int qitch = pitch + interval;
		for (unsigned j=0; j<J; j++) 
			if (_degrees[j] == PitchClass(qitch)) 
				return qitch;
	}
	return pitch;	
}


int Scale::quantiseDown(int pitch)
{
	unsigned J = _degrees.size();
	for (int interval=0; interval<12; interval++) {
		int qitch = pitch - interval;
		for (unsigned j=0; j<J; j++) {
			if (_degrees[j] == PitchClass(qitch)) {
				return qitch;
			}
		}
	}
	return pitch;	
}


int Scale::quantise(int pitch)
{
	unsigned J = _degrees.size();
	for (int interval=0; interval<12; interval++) {
		int above = pitch + interval;
		int below = pitch - interval;
		for (unsigned j=0; j<J; j++) {
			if (_degrees[j] == PitchClass(above)) {
				return above;
			} else if (_degrees[j] == PitchClass(below)) {
				return below;
			}
		}
	}
	return pitch;
}


Scale Major(PitchClass tnc)
{
	int majorSteps[7] = MAJOR_SCALE;
	return Scale(tnc,majorSteps,7);
}


Scale minor(PitchClass tnc)
{
	int minorSteps[7] = MINOR_SCALE;
	return Scale(tnc,minorSteps,7);
}


Scale MajorPentatonic(PitchClass tnc)
{
	int majorPentatonicSteps[5] = MAJOR_PENTATONIC_SCALE;
	return Scale(tnc,majorPentatonicSteps,5);
}


Scale minorPentatonic(PitchClass tnc)
{
	int minorPentatonicSteps[5] = MINOR_PENTATONIC_SCALE;
	return Scale(tnc,minorPentatonicSteps,5);
}





