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

float audioPingRate = 0.999;
float dmxPingRate = 0.9999;

#include <Bela.h>
#include <cmath>
#include <rtdk.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
//#include <deque>
//#include <Scope.h>

// Line parsing stuff
#include <string>
#include <sstream>
#include <vector>
#include <iterator>

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



//Scope scope;

#define SERIAL_BUFFER_SIZE 8
#define SERIAL_LINE_SIZE 256

int _handle;
unsigned char serialBuffer[SERIAL_BUFFER_SIZE];
unsigned char serialLine[SERIAL_LINE_SIZE+1];
unsigned int serialWriteHead;

float freq, audioPingAmp, dmxPingAmp, phase;
char cpm = 0;
char lastcpm = 0;

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


int parseGeigerReadout(std::string line) {
    int countsPerSecond = 0;
    std::vector<std::string> tokens = split(line, ',');
    if (tokens.size() > 1)
        countsPerSecond = atoi(tokens[1].c_str());

    return(countsPerSecond);
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
		        int cpm = parseGeigerReadout(geigerLine);
		        lastcpm = cpm;
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
	else if (rdlen < 0) 
        printf("Error from read: %d: %s\n", rdlen, strerror(errno));
}


float pingMagnitude(int cpm) {
    return ((float)cpm / 4.0f);
    
    
}


AuxiliaryTask ttyTask;		// Auxiliary task to read serial data

bool setup(BelaContext *context, void *userData)
{
    freq = 256.0f;
    audioPingAmp = 0.0f;
    dmxPingAmp = 0.0f;
    phase = 0.0f;
    
    //scope.setup(3, context->audioSampleRate);
    
    //initSerial("/dev/ttyACM0", B9600);
    initSerial("/dev/ttyUSB0", B9600);
    ttyTask = Bela_createAuxiliaryTask(readSerialPort, BELA_AUDIO_PRIORITY - 20, "bela-arduino-comms");
	return true;
}


void render(BelaContext *context, void *userData)
{
	//bool trigger = false;
	Bela_scheduleAuxiliaryTask(ttyTask);
	
	// loop through frames in block
	for(unsigned n = 0; n < context->audioFrames; n++) {

	    // Look for geiger counter pulse
        //int lowBias = digitalRead(context, n, 1);
        int pulse = digitalRead(context, n, 3);        
        //int highBias = digitalRead(context, n, 5);
        //scope.log(lowBias,pulse,highBias);
        
        if (pulse > 0) {
            //scope.trigger();
            //rt_printf("Boom!\n");
            audioPingAmp = 1.0f;
            dmxPingAmp = 1.0f;
            freq = 10000.0f;
        } else {
            audioPingAmp *= audioPingRate;
            freq *= audioPingRate;
            dmxPingAmp *= dmxPingRate;
        }
        
        float sample = 0.25f * ((float)rand()) / ((float)RAND_MAX) + 0.25f * sin(2.0f * 3.14159f * phase);

        // advance oscillator
        phase += freq / context->audioSampleRate;
        while (phase >= 1.0f)
            phase -= 1.0f;
        
        
        if (context->audioOutChannels == 2) {
            context->audioOut[context->audioOutChannels * n] = audioPingAmp * sample;
            context->audioOut[context->audioOutChannels * n + 1] = audioPingAmp * sample;
        } else {
    		for(unsigned ch = 0; ch < context->audioOutChannels; ch++)
	    		context->audioOut[context->audioOutChannels * n + ch] = audioPingAmp * sample;
        }
        
        analogWrite(context, n, 0, constrain(dmxPingAmp, 0.0f, 1.0f));
        
	}
}


void cleanup(BelaContext *context, void *userData)
{
    close(_handle);
}
