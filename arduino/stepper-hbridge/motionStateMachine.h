// ---------------------------------------------- Application State 
#ifndef MOTION_STATE_MACHINE_H_
#define MOTION_STATE_MACHINE_H_

#include <Arduino.h>

#define MOTION_MODE_NONE 0
#define MOTION_MODE_VEL 1 
#define MOTION_MODE_QUEUE 2 

#define MOTION_MAX_DOF 7 
#define MOTION_QUEUE_LEN 32 

// we're going to use `2.30` *and* `34.30` fixed points, 
const int32_t fp_scale = 30;

// get explicit abt which are fixed point ints, 
typedef int32_t fpint32_t;
typedef int64_t fpint64_t;

// our local state is big-dof-wise, 
typedef struct motionState_t {
  float pos[MOTION_MAX_DOF];    // position in all axes, steps
  float dir[MOTION_MAX_DOF];    // unit vector, direction of vel & accel
  float vel;                    // vel, steps/sec
  float accel;                  // accel, steps/sec 
  // float distanceToTarget;
  // float maxVel;
  // float maxAccel;
  // float twoDA;      // mostly internal, here for debuggen, works now though 
  // float vSquared;   // as above 
} motionState_t;

// typedef struct axlPlannedSegment_t {
//   // we're given these data over the network:
//   uint32_t segmentNumber = 0;               // continuity counter 
//   // boolean isLastSegment = false;            // should we expect end of queue here ? 
//   float unit[AXL_MAX_DOF];
//   float vi = 0.0F;                          // start velocity 
//   float accel = 0.0F;                       // accel rate 
//   float vmax = 0.0F;                        // max rate 
//   float vf = 0.0F;                          // end velocity 
//   float distance = 0.0F;                    // how far total 
//   // queueing flags... 
//   boolean isReady = false;                  // is... next up, needs to be executed, 
//   boolean isRunning = false;                // currently ticking, 
//   // linked list next, previous, and pos
//   axlPlannedSegment_t* next = nullptr;         // will link these... 
//   axlPlannedSegment_t* previous = nullptr;     // ... 
//   uint8_t indice = 0;                       // it can be nice to know, for list debuggen
// } axlPlannedSegment_t;

// struct for segment handoffs, 
typedef struct motionSegmentInterface_t {
  // sequencing helpers 
  uint32_t segmentNumber = 0;
  boolean isLastSegment = false;
  // values 
  float unit[MOTION_MAX_DOF];
  float vi = 0.0F;
  float accel = 0.0F;
  float vmax = 0.0F;
  float vf = 0.0F;
  float distance = 0.0F;
} motionSegmentInterface_t;

// ---------------- setup 

void motion_init(int32_t microsecondsPerIntegration);

// ---------------- run 

void motion_integrate(void);

// ---------------- solo-motor-wise 

void motion_setPositionTarget(float _targ, float _maxVel, float _maxAccel);
void motion_setVelocityTarget(float _targ, float _maxAccel);
void motion_setPosition(float _pos);

// ---------------- get actuator states 

void motion_getCurrentStates(motionState_t* statePtr);

// ---------------- queue management

void motion_addSegmentToQueue(motionSegmentInterface_t _seg);
size_t motion_getSegmentCompleteMsg(uint8_t* msg);

void motion_printDebug(void);

#endif 