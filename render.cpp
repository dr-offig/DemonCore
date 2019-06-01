/*
 ____  _____ _        _    
| __ )| ____| |      / \   
|  _ \|  _| | |     / _ \  
| |_) | |___| |___ / ___ \ 
|____/|_____|_____/_/   \_\

The platform for ultra-low latency audio and sensor processing

http://bela.io

A project of the Augmented Instruments Laboratory within the
Centre for Digital Music at Queen Mary University of London.
http://www.eecs.qmul.ac.uk/~andrewm

(c) 2016 Augmented Instruments Laboratory: Andrew McPherson,
  Astrid Bin, Liam Donovan, Christian Heinrichs, Robert Jack,
  Giulio Moro, Laurel Pardue, Victor Zappi. All rights reserved.

The Bela software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt
*/


// ---------------------------------------------- //
// ....... User Configurable Parameters ......... //

#define NP 5        // Number of different ping envelopes 
#define NL 3        // Number of DMX light outputs
#define NA 2        // Number of audio output
#define NC NL + NA  // total outputs

float main_gain = 0.25f;
float geiger_thresh_mu = 10.0f;
float geiger_thresh_sigma = 600.0f;
int geiger_thresh = 10;  // number of clicks needed before triggering
int geiger_count = 0;      // running count of geiger clicks
//float attack[NP];  
//float decay[NP];
//float sustain[NP];
//float release[NP];
//float dur[NP];
//float amp[NP];          // oscillator amp
//float f0[NP];           // oscillator freq
//float va[NP];           // vibrato amp
//float vf[NP];           // vibrate freq    
// ............................................. //




// -------------------------------------------- //
// ........... Included Libraries ............. //

// Main Bela include
#include <Bela.h>

// Serial tty includes
#include <cmath>
#include <rtdk.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

// Other Bela includes
#include <OSCServer.h>
#include <OSCClient.h>

// Line parsing includes
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <utility>

// Other STL includes
#include <algorithm>
#include <deque>

// Bespoke includes
#include "signal.h"
#include "AudioNode.h"
#include "Tonality.h"
#include "Brain.h"
// ............................................. //




// ----------------------------------------------- //
// ....... String Manipulation Utilities ......... //

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}
// ............................................. //




// ---------------------------------------------- //
// ....... Array Manipulation Utilities ......... //
template <typename D> 
void arrayCopy(D* fromArray, D* toArray, unsigned lngth) {
	for (unsigned j=0; j<lngth; j++)
		toArray[j] = fromArray[j];
}

template <typename D> 
void arraySet(D* array, D value, unsigned lngth) {
	for (unsigned j=0; j<lngth; j++)
		array[j] = value;
}
// ............................................. //




// ---------------------------------------------- //
// .......... Random Number Utilities ........... //
float urand() {
    return float(rand()) / float(RAND_MAX);
}


float bell_rand(float mu, float sigma) {
    float u1 = urand();
    float u2 = urand();
    float u3 = urand();
    float u4 = urand();
    float u5 = urand();
    float u6 = urand();
    float b = (u1 + u2 + u3 + u4 + u5 + u6) / 6.0f;
    return(mu + sigma * b);
}


float chi_rand(float mu, float sigma) {
    float b1 = bell_rand(mu,sigma);
    float b2 = bell_rand(mu, sigma);
    float b3 = bell_rand(mu, sigma);
    return sqrt(b1*b1 + b2*b2 + b3*b3);
}


// ---------------------------------------------- //
// .............. OSC Communication ............. //
AuxiliaryTask oscServerTask;
//AuxiliaryTask oscClientTask;

OSCServer oscServer;
//OSCClient oscClient;
//deque<oscpkt::Message> outbox;
int localPort = 8000;
// ............................................. //




// -------------------------------------- //
// .............. The Pings ............. //
//OneShot hits[NP];
//float freq[NP];
//float audioPingAmp[NP];
//float dmxPingAmp[NP];
//float phase[NP];

float pingMagnitude(int cntsPerSec, int cntsPerMin) {
    return (tanh(float(cntsPerSec+1) + float(cntsPerMin+1)));
}
// ...................................... //




// -------------------------------------- //
// ........... Geiger Counter ........... //

int cps = 0;
int cpm = 0;

std::pair<int,int> parseGeigerReadout(std::string line) {
    int countsPerSecond = 0;
    int countsPerMinute = 0;
    std::vector<std::string> tokens = split(line, ',');
    
    if (tokens.size() > 1)
        countsPerSecond = atoi(tokens[1].c_str());
        
    if (tokens.size() > 3)
        countsPerMinute = atoi(tokens[3].c_str());

    return(std::make_pair(countsPerSecond, countsPerMinute));
}
// ...................................... //




// ---------------------------------------------- //
// ......... Serial Port Communication .......... //

#define SERIAL_BUFFER_SIZE 8
#define SERIAL_LINE_SIZE 256

int _handle;
unsigned char serialBuffer[SERIAL_BUFFER_SIZE];
unsigned char serialLine[SERIAL_LINE_SIZE+1];
unsigned int serialWriteHead;

int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
			return -1;
    }

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}


void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		printf("Error tcsetattr: %s\n", strerror(errno));
}


int initSerial(const char *portname, int speed)
{
    printf ("Attempting to connect to %s\n", portname);
	_handle = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (_handle < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		return -1;
	} else {
		printf ("Successfully opened %s with file descriptor %d\n", portname, _handle);
	}

	set_interface_attribs(_handle, speed);
	set_mincount(_handle, 0);                /* set to pure timed read */
	
	memset(serialLine, 0, SERIAL_LINE_SIZE);
	serialLine[SERIAL_LINE_SIZE] = '\0';
	serialWriteHead = 0;
    return 0;
}


void readSerialPort() 
{
    int rdlen;
	rdlen = read(_handle, serialBuffer, SERIAL_BUFFER_SIZE);
	if (rdlen > 0) {
            
		for (int j=0; j<rdlen; j++) {
		    char c = serialBuffer[j];
		    //printf("%c",c);
		    if (c == '\n') {
		        serialWriteHead = 0;
		        std::string geigerLine((char*)serialLine);
		        std::pair<int,int> geigerCounts = parseGeigerReadout(geigerLine);
		        cps = geigerCounts.first;
		        cpm = geigerCounts.second;

		        //printf("%s --- %d\n", serialLine, cpm);
		    } else {
		        if (serialWriteHead < SERIAL_LINE_SIZE) {
		           serialLine[serialWriteHead++] = c; 
		        } else {
		            serialWriteHead = 0;
		        }
		    }
		    
		}

	} 
	//else if (rdlen < 0) 
    //    printf("Error from read: %d: %s\n", rdlen, strerror(errno));
}

AuxiliaryTask ttyTask; // Auxiliary task to read serial data
// ........................................................ //




// ----------------------------------------- //
// .............. Audio Graph  ............. //
//ExternalAudioSource* audioInNode;
//SensorInput* sensors;
ADSREnvelopeGenerator* env[NP-1];
LinearRampGenerator* ramp[NA];
LinearRampGenerator* houseLights;
//UnaryTransform* chirp[NA]; 
SineGenerator* lfo[NA];
Vibrator* vibrator[NA];
SquareGenerator* cycle[NA];
MatrixMixer* mixer;

//float mtof(float pitch) {
//    return(440.0f * pow(2.0f, (pitch - 69.0f / 12.0f)));
//}
// ........................................ //




// ------------------------------------------------ //
// .............. OSC Message receiving ............. //

// Message Parsing
int parseMessage(oscpkt::Message msg){
    
    printf("received message to: %s\n", msg.addressPattern().c_str());
    
    int a; float b; //int bar; int beat; int tick; int velocity;
        
    if (msg.match("/attack").popInt32(a).popFloat(b).isOkNoMoreArgs()) {
        env[a]->attack = b;
    } else if (msg.match("/decay").popInt32(a).popFloat(b).isOkNoMoreArgs()) {
        env[a]->decay = b;
    } else if (msg.match("/sustain").popInt32(a).popFloat(b).isOkNoMoreArgs()) {
        env[a]->sustain = b;
    } else if (msg.match("/release").popInt32(a).popFloat(b).isOkNoMoreArgs()) {
        env[a]->release = b;
    } else if (msg.match("/amp").popInt32(a).popFloat(b).isOkNoMoreArgs()) {
        mixer->setLevel(a,a,b);
    } else if (msg.match("/dur").popInt32(a).popFloat(b).isOkNoMoreArgs()) {
        env[a]->duration = b * 44100.0f;
    } else if (msg.match("/pitch").popInt32(a).popFloat(b).isOkNoMoreArgs()) {
        ramp[a]->start_value = b;
    } else if (msg.match("/va").popInt32(a).popFloat(b).isOkNoMoreArgs()) {
        lfo[a]->setDefaultInput(1,b);
    } else if (msg.match("/vf").popInt32(a).popFloat(b).isOkNoMoreArgs()) {
        lfo[a]->setDefaultInput(0,b);
    } else if (msg.match("/thresh_mu").popInt32(a).isOkNoMoreArgs()) {
        geiger_thresh_mu = a;
    } else if (msg.match("/thresh_sigma").popInt32(a).isOkNoMoreArgs()) {
        geiger_thresh_sigma = a; 
    } else if (msg.match("/gain").popFloat(b).isOkNoMoreArgs()) {
        main_gain = b; 
    }

    return b;
}



// OSC auxiliary task callback 
void receiveOSC(void *clientData)
{
	while (oscServer.messageWaiting())
        parseMessage(oscServer.popMessage());
}
// ..................................................... //




// ------------------------------------------------------ //
// ........... Main setup and render functions .......... //
bool setup(BelaContext *context, void *userData)
{
    //freq = 256.0f;
    //audioPingAmp = 0.0f;
    //dmxPingAmp = 0.0f;
    //phase = 0.0f;
    
    oscServer.setup(localPort);
    oscServerTask = Bela_createAuxiliaryTask(receiveOSC, BELA_AUDIO_PRIORITY - 30, "receive-osc", NULL);
    
    //sensors = new SensorInput(context->analogInChannels);
	mixer = new MatrixMixer(NP, NC);	
	for (u64 j=0; j<NA; j++) { // u64 a, u64 d, float s, u64 r, u64 st, u64 dur, float amplitude
	
		env[j] = new ADSREnvelopeGenerator(0, 10, 0.8f, 100, 0, context->audioSampleRate * 0.2f, 1.0f);
		cycle[j] = new SquareGenerator();
		vibrator[j] = new Vibrator();
		lfo[j] = new SineGenerator(); lfo[j]->setDefaultInput(0,4.0f); lfo[j]->setDefaultInput(1,0.1); 
		ramp[j] = new LinearRampGenerator(60.0f, 2000.0f, 0, context->audioSampleRate * 0.2f, false);
	    //chirp[j] = new UnaryTransform(&mtof);
	    //freq[j] = new ConstantGenerator(180.0f);
	    //chirp[j]->receiveConnectionFrom(ramp[j],0,0);
	    vibrator[j]->receiveConnectionFrom(ramp[j],0,0);
		vibrator[j]->receiveConnectionFrom(lfo[j],0,1);
		
		cycle[j]->receiveConnectionFrom(vibrator[j],0,0);
		cycle[j]->receiveConnectionFrom(env[j],0,1);
		mixer->receiveConnectionFrom(cycle[j],0,j);
		mixer->setLevel(j, j, 1.0f);
    } 
    
    //for (u64 j=NA; j<NP; j++) { // u64 a, u64 d, float s, u64 r, u64 st, u64 dur, float amplitude
	    
		env[NA] = new ADSREnvelopeGenerator(0, 10, 0.8f, 100, 0, context->audioSampleRate * 0.1f, 1.0f);
		mixer->receiveConnectionFrom(env[NA],0,NA);
		mixer->setLevel(NA, NA, 1.0f);
        
		env[NA+1] = new ADSREnvelopeGenerator(1000, 10, 1.0f, context->audioSampleRate * 10.0f, 0, 1000, 1.0f);
		mixer->receiveConnectionFrom(env[NA+1],0,NA+1);
		mixer->setLevel(NA+1, NA+1, 1.0f);
        
		houseLights = new LinearRampGenerator(0.0, 0.5f, 0, context->audioSampleRate * 10.0f, true);
		mixer->receiveConnectionFrom(houseLights,0,NA+2);
		mixer->setLevel(NA+2, NA+2, 1.0f);
    
    //} 
    
    //initSerial("/dev/ttyACM0", B9600);
    initSerial("/dev/ttyUSB0", B9600);
    ttyTask = Bela_createAuxiliaryTask(readSerialPort, BELA_AUDIO_PRIORITY - 20, "bela-arduino-comms");
	return true;
}


void render(BelaContext *context, void *userData)
{

	Bela_scheduleAuxiliaryTask(ttyTask);
	Bela_scheduleAuxiliaryTask(oscServerTask);
	
	uint64_t t = context->audioFramesElapsed;
	
	// loop through frames in block
	for(unsigned n = 0; n < context->audioFrames; n++, t++) {

	    // Look for geiger counter pulse
        int pulse = digitalRead(context, n, 3);        
        
        if (pulse > 0) {
            geiger_count++;
            //rt_printf("%d ",geiger_count);
            if (geiger_count >= geiger_thresh) {
                geiger_count = 0;
                //geiger_thresh = (int)chi_rand(geiger_thresh_mu, geiger_thresh_sigma);
                geiger_thresh = (int) (urand() * geiger_thresh_sigma + geiger_thresh_mu);
                rt_printf("Boom! New threshold: %d\n", geiger_thresh);
                for (int j=0; j<NA; j++) {
                    float timescale = chi_rand(0.08f,0.01f);
                    //u64 a, u64 d, float s, u64 r, u64 st, u64 dur, float amp    
                    env[j]->retrigger(env[j]->attack, env[j]->decay, env[j]->sustain,
                                        env[j]->release, t, timescale * context->audioSampleRate, pingMagnitude(cps, cpm));
                    
                    // frequency randomisation
                    float f0 = chi_rand(40.0f, 30.0f);
                    float f1 = f0 + chi_rand(0.0f, 0.1f);
                    float fa = f0; float fb = f1;
                    if (cps * 60 > cpm) { fa = f1; fb = f0; }
                    ramp[j]->retrigger(fa, fb, t, timescale * context->audioSampleRate);
                    //env[j]->retrigger(t, 1.0);
                }
                
                // First DMX
                env[NA]->retrigger(t);
                env[NA+1]->retrigger(t);
                //env[NA+2]->retrigger(t);
                houseLights->retrigger(t);
                
            }

        } else {
            //audioPingAmp *= audioPingRate;
            //freq *= audioPingRate;
            //dmxPingAmp *= dmxPingRate;
        }
        
        
        // float sample = 0.25f * ((float)rand()) / ((float)RAND_MAX) + 0.25f * sin(2.0f * 3.14159f * phase);

        // advance oscillator
        //phase += freq / context->audioSampleRate;
        //while (phase >= 1.0f)
        //    phase -= 1.0f;
        
        
        // Output
		float audioOut[NA];
		float analogOut[NL];
		arraySet(audioOut, 0.0f, NA);
		arraySet(analogOut, 0.0f, NL);
		
		// Render the audio Graph
		mixer->renderGraph(t,NULL);
		
		// audio outputs
		for (u64 k=0 ; k<NA; k++) {
		   audioWrite(context,n,k,mixer->outputSample(k) * main_gain); 
		}
		
		// analog outputs
		for (u64 l=0; l<NL; l++) {
		   analogWrite(context,n,l,mixer->outputSample(NA+l)); 
		}
		

      
	}
}


void cleanup(BelaContext *context, void *userData)
{
    close(_handle);
    delete(mixer);
	for (unsigned j=0; j<NA; j++) {
		delete(ramp[j]);
		//delete(chirp[j]);
		delete(cycle[j]);
		delete(env[j]);
		delete(lfo[j]);
		delete(vibrator[j]);
	}
	
	for (unsigned j=NA; j<NP-1; j++) {
		delete(env[j]);
	}
	
	delete(houseLights);
}
