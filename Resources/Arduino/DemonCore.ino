/*
  DMX_Master.ino - Example code for using the Conceptinetics DMX library
  Copyright (c) 2013 W.A. van der Meeren <danny@illogic.nl>.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include <Conceptinetics.h>


//
// CTC-DRA-13-1 ISOLATED DMX-RDM SHIELD JUMPER INSTRUCTIONS
//
// If you are using the above mentioned shield you should 
// place the RXEN jumper towards pin number 2, this allows the
// master controller to put to iso shield into transmit 
// (DMX Master) mode 
//
//
// The !EN Jumper should be either placed in the G (GROUND) 
// position to enable the shield circuitry 
//   OR
// if one of the pins is selected the selected pin should be
// set to OUTPUT mode and set to LOGIC LOW in order for the 
// shield to work
//


//
// The master will control 100 Channels (1-100)
// 
// depending on the ammount of memory you have free you can choose
// to enlarge or schrink the ammount of channels (minimum is 1)
//
#define DMX_MASTER_CHANNELS   3 

//
// Pin number to change read or write mode on the shield
//
#define RXEN_PIN                2
#define ledPin                 13

// Configure a DMX master controller, the master controller
// will use the RXEN_PIN to control its write operation 
// on the bus
DMX_Master        dmx_master ( DMX_MASTER_CHANNELS, RXEN_PIN );


// the setup routine runs once when you press reset:
void setup() {             

  pinMode(RXEN_PIN,OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  // Enable DMX master interface and start transmitting
  dmx_master.enable ();  
  
  // Set channel 1 - 50 @ 50%
  dmx_master.setChannelRange(1, 3, 255);
  //dmx_master.setChannelRange ( 2, 4, 127 );
}


int analogPins[3] = {A0, A1, A2};

// the loop routine runs over and over again forever:
void loop() 
{
  for (int j=0; j<3; j++) {
    int val = analogRead(analogPins[j]);
    byte dmx_val = (byte) (val / 4);
  
    dmx_master.setChannelValue(j+1, dmx_val);

    if (j==0) {
      float normalisedVal = float(dmx_val) / 255.0f;
      float squishedVal = normalisedVal * normalisedVal;
      float transformedVal = 255.0f * squishedVal;
      int outputVal = (int)transformedVal;

      analogWrite(9,outputVal);
      analogWrite(10,outputVal);
      analogWrite(11,outputVal);
    }
    
    
    
  }

  
  delay(100);
  
  //static int dimmer_val;
  
  // Keep fading channel 1 in from 0 to 100%
  //dmx_master.setChannelValue ( 2, dimmer_val++ );
  //if (dimmer_val > 255)
  //  dimmer_val = 0;  

  // if ( dimmer_val > 127 )
  //  digitalWrite ( ledPin, HIGH );
  //else
  //  digitalWrite ( ledPin, LOW );
    
  //delay ( 10 );
}
