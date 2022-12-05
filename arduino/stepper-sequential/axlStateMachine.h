// ---------------------------------------------- Application State 
#ifndef AXL_STATE_MACHINE_H_
#define AXL_STATE_MACHINE_H_

#include <Arduino.h>

// "settings"
#define AXL_QUEUE_LEN 32
#define AXL_MAX_DOF 7 

// it's all modes to me, baby 
#define AXL_MODE_ACCEL 1
#define AXL_MODE_VELOCITY 2
#define AXL_MODE_POSITION 3 
#define AXL_MODE_QUEUE 4

// queue does an await-before-start, in order to fill buffer... 
#define AXL_QUEUESTATE_EMPTY 1
#define AXL_QUEUESTATE_AWIATING_START 2
#define AXL_QUEUESTATE_RUNNING 3
#define AXL_QUEUESTATE_INCREMENTING 4 

// halt conditions, 
#define AXL_HALT_NONE 0 
#define AXL_HALT_SOFT 1 
#define AXL_HALT_CASCADE 3 
#define AXL_HALT_ACK_NOT_PICKED 4
#define AXL_HALT_MOVE_COMPLETE_NOT_PICKED 5
#define AXL_HALT_BUFFER_STARVED 6
#define AXL_HALT_OUT_OF_ORDER_ARRIVAL 7 

// struct for motion-state handoff 
typedef struct axlState_t {
  float pos;
  float vel;
  float accel;
  uint8_t mode;
  uint32_t currentSegmentNumber;
} axlState_t;

// struct for queued motion,
typedef struct axlPlannedSegment_t {
  // we're given these data over the network:
  uint32_t segmentNumber = 0;               // continuity counter 
  boolean isLastSegment = false;            // should we expect end of queue here ? 
  float unit[AXL_MAX_DOF];
  float vi = 0.0F;                          // start velocity 
  float accel = 0.0F;                       // accel rate 
  float vmax = 0.0F;                        // max rate 
  float vf = 0.0F;                          // end velocity 
  float distance = 0.0F;                    // how far total 
  // queueing flags... 
  boolean isReady = false;                  // is... next up, needs to be executed, 
  boolean isRunning = false;                // currently ticking, 
  // linked list next, previous, and pos
  axlPlannedSegment_t* next = nullptr;         // will link these... 
  axlPlannedSegment_t* previous = nullptr;     // ... 
  uint8_t indice = 0;                       // it can be nice to know, for list debuggen
} axlPlannedSegment_t;

// startup 
void axl_init(uint16_t microsecondsPerIntegration);

// integrator 
void axl_integrate(void);

// get states 
void axl_getCurrentStates(axlState_t* statePtr);

// set modal targets
void axl_setPositionTarget(float _targ, float _maxVel, float _maxAccel);
void axl_setVelocityTarget(float _targ, float _maxAccel);
void axl_setPosition(float _pos);

// operate dos queue 
boolean axl_hasQueueSpace(void);
void axl_addSegmentToQueue(axlPlannedSegment_t move);
uint16_t axl_getSegmentAckMsg(uint8_t* msg);
uint16_t axl_getSegmentCompleteMsg(uint8_t* msg);

// halt-and-latch, not implemented, but needed in fullness of lossy-link time 
void axl_halt(uint8_t haltCode);

// ---- setters ?
void axl_setActuatorIndice(uint8_t indice);

#endif 