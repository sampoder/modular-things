// C:\Users\jaker\AppData\Local\Arduino15\libraries\osap
#include "axlStateMachine.h"
#include "stepperDriver.h"
#include <osap.h>
#include <vt_endpoint.h>
#include <vp_arduinoSerial.h>
#include <core/ts.h>

// ---------------------------------------------- OSAP central-nugget 
OSAP osap("stepper");

// ---------------------------------------------- 0th Vertex: OSAP USB Serial
VPort_ArduinoSerial vp_arduinoSerial(&osap, "usbSerial", &Serial);

// ---------------------------------------------- 1th Vertex: Modal Inputs
// position- or velocity- targets... 
EP_ONDATA_RESPONSES onModalData(uint8_t* data, uint16_t len){

  return EP_ONDATA_ACCEPT;
}

Endpoint modalInputsEndpoint(&osap, "modalIngest", onModalData);

// ---------------------------------------------- 2nd Vertex: Queue Inputs 
// new queue-moves, and WAIT if full  
EP_ONDATA_RESPONSES onQueueData(uint8_t* data, uint16_t len){

  return EP_ONDATA_ACCEPT;
}

Endpoint queueInputsEndpoint(&osap, "queueIngest", onQueueData);

// ---------------------------------------------- 3rd Vertex: Motion State Read 
// queries only, more or less, so
EP_ONDATA_RESPONSES onMotionStateData(uint8_t* data, uint16_t len){ return EP_ONDATA_REJECT; }
boolean beforeMotionStateQuery(void);
Endpoint stateEndpoint(&osap, "motionState", onMotionStateData, beforeMotionStateQuery);
uint8_t stateData[17];
boolean beforeMotionStateQuery(void){
  axlState_t state;
  axl_getCurrentStates(&state);
  uint16_t wptr = 0;
  ts_writeFloat32(state.pos, stateData, &wptr);
  ts_writeFloat32(state.vel, stateData, &wptr);
  ts_writeFloat32(state.accel, stateData, &wptr);
  stateData[wptr ++] = state.mode;
  ts_writeUint32(state.currentSegmentNumber, stateData, &wptr);
  stateEndpoint.write(stateData, wptr);
  return true;
}

// ---------------------------------------------- 3rd Vertex: Set Current Position 
EP_ONDATA_RESPONSES onPositionSetData(uint8_t* data, uint16_t len){
  // should do maxAccel, maxVel, and (optionally) setPosition 
  // upstream should've though of this, so, 
  uint16_t rptr = 0;
  float pos = ts_readFloat32(data, &rptr);
  axl_setPosition(pos);
  return EP_ONDATA_ACCEPT;
}

Endpoint positionSetEndpoint(&osap, "setPosition", onPositionSetData);

// ---------------------------------------------- 4th Vertex: Settings catch-all, 

EP_ONDATA_RESPONSES onSettingsData(uint8_t* data, uint16_t len){
  // it's just <cscale> for the time being, 
  uint16_t rptr = 0;
  float cscale = ts_readFloat32(data, &rptr);
  stepper_setCurrentScale(cscale);
  return EP_ONDATA_ACCEPT;
}

Endpoint settingsEndpoint(&osap, "settings", onSettingsData);

// ---------------------------------------------- 5th Vertex: Limit / Switch Output... non-op at the moment, 

// fair warning, this is unused at the moment... and not set-up, 
// also the limit pin is config'd to look at the interrupt on a scope at the moment, see motionStateMachine.cpp 
Endpoint buttonEndpoint(&osap, "buttonState");

void setup() {
  Serial.begin(0);
  // ~ important: the stepper code initializes GCLK4, which we use as timer-interrupt
  // in the motion system, so it aught to be initialized first ! 
  stepper_init();
  // another note on the motion system:
  // at the moment, we have a relatively small absolute-maximum speed: say the integrator interval is 250us, 
  // we have 0.00025 seconds between ticks, for a max of 4000 steps / second... 
  // we are then microstepping at 1/4th steps, for 800 steps per motor revolution, (from a base of 200)
  // meaning we can make only 5 revs / sec, or 300 rippums (RPM), 
  // with i.e. a 20-tooth GT2 belt, we have 40mm of travel per revolution, making only 200mm/sec maximum traverse 
  // this is not pitiful, but not too rad, and more importantly is that we will want to communicate these limits 
  // to users of the motor - so we should outfit a sort of settings-grab function, or something ? 
  axl_init(1000);
  // uuuh... 
  osap.init();
  // run the commos 
  vp_arduinoSerial.begin();
}

void loop() {
  // do graph stuff
  osap.loop();
  // check for messages from motion system ?
}