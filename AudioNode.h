/***** AudioNode.h *****/

#ifndef __AUDIO_NODE__
#define __AUDIO_NODE__

#include <Bela.h>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <deque>
#include "RingBuffer.h"

#define u64 uint64_t

using namespace std;

class AudioNode;

class AudioConnection
{
public:
	AudioConnection() : _fromNode(NULL), _fromChannel(0), _toChannel(0) {}
	AudioConnection(AudioNode* fromNode, u64 fromChannel, u64 toChannel) : _fromNode(fromNode), _fromChannel(fromChannel), _toChannel(toChannel) {}
	~AudioConnection() {}
	AudioConnection(const AudioConnection& ac) { _fromNode = ac._fromNode; _fromChannel = ac._fromChannel; _toChannel = ac._toChannel; } 
	
	AudioNode* fromNode() { return _fromNode; }
	inline const u64& fromChannel() { return _fromChannel; }
	inline const u64& toChannel() { return _toChannel; }
	
protected:	
	AudioNode* _fromNode;
	u64 _fromChannel;
	u64 _toChannel;
};


class AudioNode
{
public:
	AudioNode() : _inputBuffer(NULL), _outputBuffer(NULL), _defaultInputBuffer(NULL), _inputChannels(0), _outputChannels(0), _inputConnections(), _lastFrameProcessed(0) {}
	AudioNode(u64 inChannels, u64 outChannels);
	virtual ~AudioNode() { free(_inputBuffer); free(_outputBuffer); free(_defaultInputBuffer); }
	AudioNode(const AudioNode&);
	
	virtual bool active(u64 frame) { return true; }	
	inline u64 getNumInputChannels() const { return _inputChannels; }
	inline u64 getNumOutputChannels() const { return _outputChannels; }
	virtual void setNumInputChannels(const u64 numChannels);
	virtual void setNumOutputChannels(const u64 numChannels);
	void renderGraph(u64 frame, void* clientData);
	virtual void processFrame(u64 frame, void* clientData);
		//{ for (u64 i=0; i < _outputChannels; i++) _outputBuffer[i]=0.0f;  }
	inline float outputSample(u64 channel) { return _outputBuffer[channel]; }
	int indexOfConnection(const u64 channel);
	AudioConnection* whatIsConnectedTo(const u64 channel);
	void receiveConnectionFrom(AudioNode* fromNode, u64 fromChannel, u64 toChannel);
	void removeConnection(u64 toChannel);
	void setDefaultInput(u64 inChannel,float value);
	virtual void initDefaults();
	//virtual float defaultFor(u64 chan) { printf("Setting AudioNode default on channel %lld to 0.0f\n", chan); return 0.0f; }
	
protected:
	float* _inputBuffer;
	float* _outputBuffer;
	float* _defaultInputBuffer;
	u64 _inputChannels;
	u64 _outputChannels;
	vector<AudioConnection*> _inputConnections;
	u64 _lastFrameProcessed;
};


class SineGenerator : public AudioNode
{
public:
	SineGenerator() : AudioNode(2,1), _phase(0.0), _sampleRate(44100.0) { initDefaults(); }
	virtual ~SineGenerator() {}
	SineGenerator(const SineGenerator& sg) : AudioNode(sg), _phase(sg._phase), _sampleRate(sg._sampleRate) { initDefaults(); }
	
	virtual void processFrame(u64 frame, void* clientData);
	virtual void initDefaults();
	
private:
	double _phase;
	double _sampleRate;
};


class SawGenerator : public AudioNode
{
public:
	SawGenerator() : AudioNode(2,1), _phase(0.0), _sampleRate(44100.0) { initDefaults(); }
	virtual ~SawGenerator() {}
	SawGenerator(const SawGenerator& sg) : AudioNode(sg), _phase(sg._phase), _sampleRate(sg._sampleRate) { initDefaults(); }
	
	virtual void processFrame(u64 frame, void* clientData);
	virtual void initDefaults();

private:
	double _phase;
	double _sampleRate;
};


class SquareGenerator : public AudioNode
{
public:
	SquareGenerator() : AudioNode(2,1), _phase(0.0), _sampleRate(44100.0) { initDefaults(); }
	virtual ~SquareGenerator() {}
	SquareGenerator(const SquareGenerator& sg) : AudioNode(sg), _phase(sg._phase), _sampleRate(sg._sampleRate) { initDefaults(); }
	
	virtual void processFrame(u64 frame, void* clientData);
	virtual void initDefaults();

private:
	double _phase;
	double _sampleRate;
};




class ConstantGenerator : public AudioNode
{
public:
	ConstantGenerator(float val) : AudioNode(0,1), value(val) {}
	virtual ~ConstantGenerator() {}
	ConstantGenerator(const ConstantGenerator& cg) : AudioNode(cg) { value = cg.value; } 
	
	virtual void processFrame(u64 frame, void *clientData);
	virtual void initDefaults();
	float value;
	
};


class WhiteNoiseGenerator : public AudioNode
{
public:
	WhiteNoiseGenerator(float val) : AudioNode(0,1) {}
	virtual ~WhiteNoiseGenerator() {}
	WhiteNoiseGenerator(const WhiteNoiseGenerator& cg) : AudioNode(cg) { } 
	
	virtual void processFrame(u64 frame, void *clientData);
	
};



class InitialImpulseGenerator : public AudioNode
{
public:
	InitialImpulseGenerator() : AudioNode(0,1), _triggered(false) { }
	virtual ~InitialImpulseGenerator() {}
	InitialImpulseGenerator(const InitialImpulseGenerator& cg) : AudioNode(cg), _triggered(false) { } 
	
	virtual void processFrame(u64 frame, void *clientData);

private:
	bool _triggered;
};



class ADSREnvelopeGenerator : public AudioNode 
{
public:
	ADSREnvelopeGenerator(u64 a, u64 d, float s, u64 r, u64 st, u64 dur, float amp);
	virtual ~ADSREnvelopeGenerator() {}
	ADSREnvelopeGenerator(const ADSREnvelopeGenerator& adsr);
	
	virtual bool active(u64 frame);
	virtual void processFrame(u64 frame, void *clientData);
		
	void retrigger(u64 a, u64 d, float s, u64 r, u64 st, u64 dur, float amp);
	void retrigger(u64 st, float amp);
	void retrigger(u64 st);
	
	u64 attack;
	u64 decay;
	float sustain;
	u64 release;
	u64 start_time;
	u64 duration;
	float amplitude;

private:
	bool _crossFading;
	u64 _crossFadeDuration;
	u64 _crossFadeTime;
	float _holdValue;
	
};


class LinearRampGenerator : public AudioNode 
{
public:
	LinearRampGenerator(float a, float b, u64 st, u64 dur, bool stck=false);
	virtual ~LinearRampGenerator() {}
	LinearRampGenerator(const LinearRampGenerator& ramp);
	
	virtual bool active(u64 frame);
	virtual void processFrame(u64 frame, void *clientData);
		
	void retrigger(float a, float b, u64 st, u64 dur);
	void retrigger(u64 st, float a);
	void retrigger(u64 st);
	
	u64 start_value;
	u64 end_value;
	u64 start_time;
	u64 duration;
	bool sticky;
	
private:
	bool _crossFading;
	u64 _crossFadeDuration;
	u64 _crossFadeTime;
	float _holdValue;
	
};


class ExpRampGenerator : public AudioNode 
{
public:
	ExpRampGenerator(float a, float r, u64 st, u64 dur, bool stck=false);
	virtual ~ExpRampGenerator() {}
	ExpRampGenerator(const ExpRampGenerator& ramp);
	
	virtual bool active(u64 frame);
	virtual void processFrame(u64 frame, void *clientData);
		
	void retrigger(float a, float r, u64 st, u64 dur);
	void retrigger(u64 st, float a);
	void retrigger(u64 st);
	
	u64 start_value;
	u64 rate;
	u64 start_time;
	u64 duration;
	bool sticky;
	
private:
	bool _crossFading;
	u64 _crossFadeDuration;
	u64 _crossFadeTime;
	float _holdValue;
	
};







typedef float (*UnaryFunc)(float);

class UnaryTransform : public AudioNode
{

public:
    UnaryTransform(UnaryFunc func) : AudioNode(1,1), _func(func) {}
    virtual ~UnaryTransform() {}
    UnaryTransform(const UnaryTransform& ut) : AudioNode(ut), _func(ut._func) {}

    virtual void processFrame(u64 frame, void* clientData);
    
private:
    UnaryFunc _func;

};



class StereoMixer : public AudioNode
{
public:
	StereoMixer();
	StereoMixer(u64 numInputChannels);
	virtual ~StereoMixer();
	StereoMixer(const StereoMixer& sm);
	
	virtual void processFrame(u64 frame, void *clientData);
	virtual void setNumInputChannels(const u64 numChannels);
	void setPan(u64 channel,float value);

private:	
	
	float _c;
	float* _pan;
	float* _a;
	float* _b;
};



class MatrixMixer : public AudioNode
{
public:
	MatrixMixer();
	MatrixMixer(u64 numInputChannels, u64 numOutputChannels);
	virtual ~MatrixMixer();
	MatrixMixer(const MatrixMixer& sm);
	
	virtual void processFrame(u64 frame, void *clientData);
	virtual void setNumInputChannels(const u64 numChannels);
	virtual void setNumOutputChannels(const u64 numChannels);
	float getLevel(u64 inChannel, u64 outChannel);
	void setLevel(u64 inChannel, u64 outChannel, float value);

private:	
	
	float* _levelMatrix;
};



class Multiplier : public AudioNode
{
public:
	Multiplier() : AudioNode(2,1) { initDefaults(); }
	Multiplier(u64 numInputChannels) : AudioNode(numInputChannels,1) { initDefaults(); }
	virtual ~Multiplier() {}
	Multiplier(const Multiplier& mp) : AudioNode(mp) { initDefaults(); }
	
	virtual void processFrame(u64 frame, void* clientData);
	virtual void initDefaults();
};


class Adder : public AudioNode
{
public:
	Adder() : AudioNode(2,1) {}
	Adder(u64 numInputChannels) : AudioNode(numInputChannels,1) {}
	virtual ~Adder() {}
	Adder(const Adder& ad) : AudioNode(ad) {}
	
	virtual void processFrame(u64 frame, void* clientData);
};


class Affine : public AudioNode
{
public:
	Affine() : AudioNode(3,1) {}
	virtual ~Affine() {}
	Affine(const Affine& af) : AudioNode(af) {}
	
	virtual void processFrame(u64 frame, void* clientData);
	virtual void initDefaults();
};


class Vibrator : public AudioNode
{
public:
	Vibrator() : AudioNode(2,1) { initDefaults(); }
	virtual ~Vibrator() {}
	Vibrator(const Vibrator& af) : AudioNode(af) { initDefaults(); }
	
	virtual void processFrame(u64 frame, void* clientData);
	virtual void initDefaults();
	
};


class Splitter : public AudioNode
{
public:
	Splitter() : AudioNode(1,2) {}
	Splitter(u64 numOutputChannels) : AudioNode(1,numOutputChannels) {}
	virtual ~Splitter() {}
	Splitter(const Splitter& sp) : AudioNode(sp) {}
	
	virtual void processFrame(u64 frame, void* clientData);
};



/************** effects *****************/

class DelayLine : public AudioNode
{
public:
	DelayLine(unsigned n) : AudioNode(1,1), _pipeline(n, 0.0f) {}
	virtual ~DelayLine() {}
	DelayLine(const DelayLine& dl) : AudioNode(dl), _pipeline(dl._pipeline) {}
	
	virtual void processFrame(u64 frame, void* clientData);

private:
	ShiftingPool<float> _pipeline;
	
};


class FIRFilter : public AudioNode
{
public:
	FIRFilter(unsigned n);
	virtual ~FIRFilter();
	FIRFilter(const FIRFilter& fl);
	
	virtual void processFrame(u64 frame, void* clientData);
	void setTap(unsigned t, float h);
	void print();
	
private:
	ShiftingPool<float> _pipeline;
	float* _filterCoefficients;
	unsigned _order;
	double _sampleRate;
};



/*************** external input ****************/

class ExternalAudioSource : public AudioNode
{
public:
	ExternalAudioSource() : AudioNode(0,2) {}
	ExternalAudioSource(u64 inChans) : AudioNode(0,inChans) {}
	virtual ~ExternalAudioSource() {}
	ExternalAudioSource(const ExternalAudioSource& es) : AudioNode(es) {}
	
	virtual void processFrame(u64 frame, void *clientData) {}
	void updateBufferFromSource(float *inData, u64 inChans);

};


class SensorInput: public AudioNode
{
public:
	SensorInput() : AudioNode(0,1) {}
	SensorInput(u64 chans) : AudioNode(0,chans) {}
	virtual ~SensorInput() {}
	SensorInput(const SensorInput& si) : AudioNode(si) {}
	
	virtual void processFrame(u64 frame, void *clientData) {}
	void updateBufferFromSource(float* inData, u64 inChans);
	
};


#endif
