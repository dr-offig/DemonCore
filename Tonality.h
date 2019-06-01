/***** Tonality.h *****/

#ifndef __TONALITY__
#define __TONALITY__

#include <Bela.h>
#include <list>
#include <vector>
#include <math.h>

#define MAJOR_SCALE {2,2,1,2,2,2,1}
#define MINOR_SCALE {2,1,2,2,2,1,2}
#define MAJOR_PENTATONIC_SCALE {2,2,3,2,3}
#define MINOR_PENTATONIC_SCALE {3,2,2,3,2}

using namespace std;

// Modulo function that always returns a positive value
unsigned modulo(int x, int b);
unsigned mod12(int x);

inline float mtof(int m)
{
	return float(440.0 * (pow (2.0, (double(m-69)/12.0))));
}


class PitchClass
{
public:
	PitchClass() : doremi(0) {}
	PitchClass(int pitch) : doremi(mod12(pitch)) {}
	~PitchClass() {}
	PitchClass(const PitchClass& pc) : doremi(pc.doremi) {};
	friend inline bool operator==(const PitchClass& l, const PitchClass& r) { return l.doremi == r.doremi; }
	friend inline PitchClass operator+(const PitchClass& l, const PitchClass& r) { return PitchClass(l.doremi + r.doremi); }
	friend inline PitchClass operator+(const PitchClass& l, const int& r) { return PitchClass(l.doremi + r); }
	friend inline PitchClass operator+(const int& l, const PitchClass& r) { return PitchClass(l + r.doremi); }
	friend inline int operator-(const PitchClass& l, const PitchClass& r) { return int(l.doremi) - int(r.doremi); }
	
	int pitchInOctave(int octave);
	unsigned doremi;

};


class Scale
{
public:

	Scale();
	Scale(PitchClass&,int*,unsigned);
	~Scale() {}
	Scale(const Scale&);
	
	void transposeTo(PitchClass& pc);
	PitchClass transposeBy(int& semitones); 
	int pitchInOctave(PitchClass& pc,int octave);
	int quantiseUp(int pitch);
	int quantiseDown(int pitch);
	int quantise(int pitch);
	PitchClass pitchClass(int degree) { return _degrees[modulo(degree,_degrees.size())]; }
	int pitch(int degree,int octave) { return pitchClass(degree).pitchInOctave(octave); }

protected:	
	PitchClass _tonic;			
	list<int> _steps;
	vector<PitchClass> _degrees;
	
};


Scale Major(PitchClass tnc);
Scale minor(PitchClass tnc);
Scale MajorPentatonic(PitchClass tnc);
Scale minorPentatonic(PitchClass tnc);

#endif