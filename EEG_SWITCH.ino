/*
Photic Driven Switch Firmware

Date created:	2018-07-17
Author:			Makers' Making Change
Revisions:		C.Zeng (2018/07/17) file created
				C.Zeng (2018/08/01) implemented recursive method of the library in loop(), channelVal = ... ... ...
*/

#include <OpenBCI_Wifi_Master_Definitions.h>

#include <OpenBCI_Wifi_Master.h>
#include <OpenBCI_Ganglion_Library.h>
#include <Biquad.h>

#define RELAY_PIN 11						// pin used to control relay
#define RELAY_HOLD_TIME 1					// in second(s)
#define EXT_LED 12
#define FILTER_Q 0.5f        				// 0.707 (Butterworth) when critically damped
#define NOTCH_Q 4.0f 						// sharp notch
#define BP_Q 2.0f							// step slope
#define PEAK_GAIN_DB 0.0f					// don't want any gain in banpass filter
#define MICROVOLTS_PER_COUNT 0.02235174f	//
#define HP_CUTOFF 0.5f						// 
#define NOTCH_FREQ 60.0f 					// power line frequency
#define TARGET_FREQUENCY 9.0f 				// the frequency that to be detected
#define CHANNEL_LOW 5.0f					// when then activity in such channel is low

const int NUM_CHANNEL = 2;    // number of signals needed from the brains
const int SAMPLE_RATE = 250;  // sampling rate used by OpenBCI

bool frequencyMatch = false;

float channelPhase[NUM_CHANNEL][SAMPLE_RATE] = {};		// 2d array to store channel data

Biquad stopDC_filter(bq_type_highpass, HP_CUTOFF / SAMPLE_RATE, FILTER_Q, PEAK_GAIN_DB);
Biquad notch_filter(bq_type_notch, NOTCH_FREQ / SAMPLE_RATE, NOTCH_Q, PEAK_GAIN_DB);
Biquad AHP_bandpass_filter(bq_type_bandpass, TARGET_FREQUENCY / SAMPLE_RATE, BP_Q, PEAK_GAIN_DB);
stopDC_filter.calcBiquad();
notch_filter.calcBiquad();
AHP_bandpass_filter.calcBiquad();

void setup(){
	// Bring up the Ganglion
  	ganglion.initialize();
    attachPinInterrupt(MCP_DRDY, MCP_ISR, LOW);
  
  	pinMode(RELAY_PIN, OUTPUT);
  	pinMode(EXT_LED, OUTPUT);

  	// initiating indicator
  	for (int i = 0; i < 5; i ++){
  		digitalWrite(EXT_LED, HIGH);
  		delay(250);
  		digitalWrite(EXT_LED, LOW);
  		delay(250);
  	}
}

void loop(){

	// fetching channel date into a phase for future analyst
	for (int i = 0; i < SAMPLE_RATE; i++){
		ganglion.updateMCPdata();
		for (int j = 0; j < NUM_CHANNEL; j++){
			channelPhase[j][i] = ganglion.channelData[j];
		}
	}

	// frequency detecting
	float EEGuv[NUM_CHANNEL];
	for (int i = 0; i < NUM_CHANNEL; i++){
		float channelVal = 0.0f;
		float channelSum = 0.0f;
		for (int j = 0; j < SAMPLE_RATE; j++){
			channelVal = channelPhase[i][j];
			// recursive functions
			channelVal = stopDC_filter.process(stopDC_filter.process(channelVal));				// applying DC-blocking filter
			channelVal = notch_filter.process(notch_filter.process(channelVal));				// applying 60Hz notch filter
      		channelVal = AHP_bandpass_filter.process(AHP_bandpass_filter.process(channelVal)); 	// applying band pass filter
			channelSum += channelVal * channelVal;												// scaling the data
		}
		EEGuv[i] = (sqrt(abs(channelSum / SAMPLE_RATE)) * MICROVOLTS_PER_COUNT);
	}

	for (int i = 0; i < NUM_CHANNEL; i++) if (EEGuv[i] > CHANNEL_LOW) frequencyMatch = true;
	if (frequencyMatch){
		digitalWrite(RELAY_PIN, HIGH);
		delay(RELAY_HOLD_TIME * 1000);
		digitalWrite(RELAY_PIN, LOW);
		frequencyMatch = false;
	}
	else digitalWrite(RELAY_PIN, LOW);
}

int MCP_ISR(uint32_t dummyPin) { // gotta have a dummyPin...

  ganglion.MCP_dataReady = true;
  ganglion.sampleCounter++;

  return 0; // gotta return nothing, somehow...
}