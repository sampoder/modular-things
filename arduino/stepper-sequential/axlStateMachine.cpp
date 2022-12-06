#include "axlStateMachine.h"
#include "stepperDriver.h"
#include <core/ts.h>

// ---------------------------------------------- stopping criteria... 
#define POS_EPSILON 0.01F
#define VEL_EPSILON 1.0F
#define TICK_INTERVAL 1000.0F

// ---------------------------------------------- ~ various 
// delT is re-calculated when we init w/ a new microsecondsPerIntegration 
float delT = 0.001F;
// not recalculated... or settings-adjustable, yet, 
uint8_t microsteps = 4; // and note (!) this *is not* "microstepping" as in 1/n, it's n/16, per our LUTS 
// init-once values we'll use in the integrator 
volatile float delta = 0.0F;
volatile float stepModulo = 0.0F;
volatile float distanceToTarget = 0.0F;
volatile float stopDistance = 0.0F;
volatile float absMaxVelocity = 10.0F;

// ---------------------------------------------- ~ core states 
// states (units are steps, 1=1 ?) 
volatile uint8_t mode = AXL_MODE_QUEUE;
volatile float pos = 0.0F;                // current position 
volatile float vel = 0.0F;                // current velocity 
volatile float accel = 0.0F;              // current acceleration 
volatile float segDistance = 0.0F;
volatile float segDelta = 0.0F;
volatile float segStopDistance = 0.0F;
volatile float segVel = 0.0F;
volatile float segAccel = 0.0F;

// ---------------------------------------------- queues 
volatile uint8_t queueState = AXL_QUEUESTATE_EMPTY;
axlPlannedSegment_t queue[AXL_QUEUE_LEN];
volatile axlPlannedSegment_t* queueHead;   // ingest here, 
volatile axlPlannedSegment_t* queueTail;   // operate here, 
uint32_t queueStartDelayMS = 500; // time between empty-queue to start-of-first, to allow ingestion 
volatile uint32_t queueStartTime = 0;      // when to start ? 
volatile uint32_t nextSegmentNumber = 0;   // what's the next segment we expect ? to guard out-of-order arrival 

// ---------------------------------------------- we have some outbound messages
// for sure this is not RAM-friendly, wherps 
uint8_t segmentAckMsg[128];
uint16_t segmentAckMsgLen = 0;
uint8_t segmentCompleteMsg[128];
volatile uint16_t segmentCompleteMsgLen = 0;
uint8_t haltMsg[128];
uint16_t haltMsgLen = 0;

// ---------------------------------------------- axl dof-to-motor linkage, 
uint8_t ourActuatorIndice = 0;

void axl_setActuatorIndice(uint8_t indice){
  if(indice >= AXL_MAX_DOF) indice = AXL_MAX_DOF - 1;
  ourActuatorIndice = indice;
}

// this is the "limit" pin, currently used as a debug, 
#define PIN_TICK 22

// ---------------------------------------------- queue init 
void queue_init(void){
  // set unit vectors to zero, just in case we stumble into one (?) 
  for(uint8_t i = 0; i < AXL_QUEUE_LEN; i ++){
    for(uint8_t a = 0; a < AXL_MAX_DOF; a ++){
      queue[i].unit[a] = 0.0F;
    }
  }
  // link the ringbuffer, 
  for(uint8_t i = 0; i < AXL_QUEUE_LEN; i ++){
    queue[i].indice = i;
    if(i != AXL_QUEUE_LEN - 1) queue[i].next = &(queue[i+1]);
    if(i != 0) queue[i].previous = &(queue[i-1]);
  }
  // wrap-around link cases,
  queue[0].previous = &(queue[AXL_QUEUE_LEN - 1]);
  queue[AXL_QUEUE_LEN - 1].next = &(queue[0]);
  // init head & tail ptrs... 
  queueHead = &(queue[0]);  // where to write-in, 
  queueTail = &(queue[0]);  // which is ticking along... 
}

// ---------------------------------------------- system init 
// for the timer inits, 
// s/o to http://academy.cba.mit.edu/classes/output_devices/servo/hello.servo-registers.D11C.ino 
// s/o also to https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f 
void axl_init(uint16_t microsecondsPerIntegration){
  // startup yonder queueue 
  queue_init();
  // before we get into hardware, let's consider our absolute-maximums;
  // here's our delta-tee:
  delT = (float)(microsecondsPerIntegration) / 1000000.0F;
  // we absolutely cannot step more than one tick-per-integration cycle, 
  // since we are in one-step-per-unit land, it means our absMax is just 1/delT, 
  absMaxVelocity = 1.0F / delT; 
  // that's it - we can get on with the hardware configs 
  PORT->Group[0].DIRSET.reg = (uint32_t)(1 << PIN_TICK);
  // states are all initialized already, but we do want to get set-up on a timer interrupt, 
  // here we're using GCLK4, which I am assuming is set-up already / generated, in the 
  // stepper module, which uses it for PWM outputs ! 
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | 
                      GCLK_CLKCTRL_GEN_GCLK4 |
                      GCLK_CLKCTRL_ID_TC4_TC5;
  while(GCLK->STATUS.bit.SYNCBUSY);
  // now we want to unmask the TC5, 
  PM->APBCMASK.reg |= PM_APBCMASK_TC5;
  // set timer modes / etc, 
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 |
                            TC_CTRLA_WAVEGEN_MFRQ |
                            TC_CTRLA_PRESCALER_DIV8; // div/8 on 48mhz clock, so 6MHz base, 6-ticks-per-microsecond, 
  while(TC5->COUNT16.STATUS.bit.SYNCBUSY);
  // enable the interrupt,
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 1); // hmmm 
  NVIC_EnableIRQ(TC5_IRQn);
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  // set la freqweenseh
  TC5->COUNT16.CC[0].reg = 6 * microsecondsPerIntegration;
  // and enable it, 
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while(TC5->COUNT16.STATUS.bit.SYNCBUSY);
}

void TC5_Handler(void){
  PORT->Group[0].OUTSET.reg = (uint32_t)(1 << PIN_TICK);  // marks interrupt entry, to debug 
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; // clear the interrupt
  axl_integrate(); // do the motion system integration, 
  PORT->Group[0].OUTCLR.reg = (uint32_t)(1 << PIN_TICK);  // marks exit 
}

void axl_integrate(void){
  // check if we have / are within a valid move, 
  if(!(queueTail->isReady)) return;
  // past this point we have at least one move to make, 
  switch(queueState){
    case AXL_QUEUESTATE_RUNNING: // we were previously running, carry on:
      break;
    case AXL_QUEUESTATE_EMPTY: // we were previously empty, this is new, setup the delay: 
      segDistance = 0.0F;
      queueState = AXL_QUEUESTATE_AWIATING_START;
      queueStartTime = millis() + queueStartDelayMS;
      return;
    case AXL_QUEUESTATE_AWIATING_START:  // we have setup the delay, are now waiting... 
      if(millis() > queueStartTime){
        queueState = AXL_QUEUESTATE_INCREMENTING;
      }
      return;
    case AXL_QUEUESTATE_INCREMENTING:
      // we've just collected a new tail, so we should do 
      segVel = queueTail->vi; 
      segAccel = queueTail->accel; // although this statemachine decides this again... 
      queueState = AXL_QUEUESTATE_RUNNING;
      // OSAP::debug("vi:\t" + String(queueTail->vi, 2) + "\tvf:\t" + String(queueTail->vf, 2) + "\tdst:\t" + String(queueTail->distance, 2));
      break;
    default:
      break;
  }
  // we want to know if it's time to start stopping... so, we have 
  // vf^2 = vi^2 + 2ad 
  // vf^2 - vi^2 = 2ad
  // note that accel is always +ve in the move, and here we are considering deceleration, so it flips 
  // stopDistance will be -ve when vf > vi, i.e. when the move is accelerating... 
  segStopDistance = ((queueTail->vf * queueTail->vf) - (segVel * segVel)) / (2 * queueTail->accel);
  // we have a distance-to-end: total length of move - distance traversed along move so far, 
  segDelta = queueTail->distance - segDistance;
  // if we are just past stopping distance, we should start slowing down 
  // (ideally we would anticipate the next integral, so that we don't overshoot *at all*)
  if(stopDistance > abs(segDelta)){
    segAccel = -queueTail->accel;
  } else {
    segAccel = queueTail->accel;
  }
  // OSAP::debug("stopdist:\t" + String(stopDistance, 2) + "\t" + String(segTail->distance - segDistance));
  // then we want to integrate our linear velocity,
  segVel += segAccel * delT;
  // no negative rates, that would be *erroneous* and also *bad* 
  if(segVel < -0.001F){
    segVel = 0.0F;
    // OSAP::error("negative rate " + String(segVel), MEDIUM);
    return;
  }
  // and hit vmax ceilings, 
  if(segVel > queueTail->vmax) segVel = queueTail->vmax;
  // integrate per-segment position, 
  segDistance += segVel * delT;
  // now do integrations over space ? for our "picked" axis, 
  // our velocity on this axis is yonder:
  vel = segVel * queueTail->unit[ourActuatorIndice];
  // we have an on-axis position delta:
  segDelta = vel * delT;
  // we stash it in our position state, 
  pos += segDelta;
  // and we use that to operate our stepper: 
  stepModulo += segDelta;
  if(stepModulo > 1.0F){
    stepper_step(microsteps, true);
    stepModulo -= 1.0F;
  } else if (stepModulo < -1.0F){
    stepper_step(microsteps, false);
    stepModulo += 1.0F;
  }
  // check check, 
  // OSAP::debug("vel: " + String(segVel, 2) + " stopDist: " + String(stopDistance, 2) + " dist:" + String(segDistance, 2));
  // -------------------------------------------- Check Segment Completion 
  // are we done? goto the next move,
  if(segDistance >= queueTail->distance){
    // this move gets a reset, so queue observers know it's "empty" 
    queueTail->isReady = false;
    queueTail->isRunning = false;
    // make a segment-complete-ack, 
    // was previous ack picked up in time ? bad if not 
    // also could implement windowed... so, just write most-recently-completed move
    // TODO should do that above, but consider also the bus-use case, how do they work together ? 
    if(segmentCompleteMsgLen != 0){
      axl_halt(AXL_HALT_MOVE_COMPLETE_NOT_PICKED);
    }
    // otherwise carry on, 
    segmentCompleteMsgLen = 0;
    uint16_t scmWPtr = 0;
    // segment #, and our actuator ID... 
    ts_writeUint32(queueTail->segmentNumber, segmentCompleteMsg, &scmWPtr);
    segmentCompleteMsgLen = scmWPtr;
    // that's it, the ack will be picked up... 
    // before we increment, stash this extra distance into the next segment... 
    segDistance = segDistance - queueTail->distance;
    // was it the last ?
    // boolean wasLastMove = queueTail->isLastSegment;
    // now increment the pointer, 
    queueTail = queueTail->next;
    // is that ready ? then grab another, if not, set moves-complete, 
    if(!queueTail->isReady){
      // if this is true and our velocities != 0, we are probably starved:
      // if(!wasLastMove) axl_halt(AXL_HALT_BUFFER_STARVED);
      // we're empty now, so 
      queueState = AXL_QUEUESTATE_EMPTY;
      // and set velocity to 0, 
      vel = 0.0F;
      // if we're empty, don't goto pickup, just bail... 
      return; 
    } else {
      queueState = AXL_QUEUESTATE_INCREMENTING;
      queueTail->isRunning = true;
    }
  } // end is-move-complete section, 
  // -------------------------------------------- Integrate -> States... 
  // // set our accel based on modal requests, 
  // switch(mode){
  //   case MOTION_MODE_POS:
  //     distanceToTarget = posTarget - pos;
  //     stopDistance = (vel * vel) / (2.0F * maxAccel);
  //     if(abs(distanceToTarget - delta) < POS_EPSILON && abs(vel) < VEL_EPSILON){
  //       // zero out and don't do any phantom motion 
  //       delta = 0.0F;
  //       vel = 0.0F;
  //       accel = 0.0F;
  //       return; 
  //     }
  //     if(stopDistance >= abs(distanceToTarget)){    // if we're going to overshoot, deccel:
  //       if(vel <= 0.0F){                            // if -ve vel,
  //         accel = maxAccel;                         // do +ve accel, 
  //       } else {                                    // if +ve vel, 
  //         accel = -maxAccel;                        // do -ve accel, 
  //       }
  //     } else {
  //       if(distanceToTarget > 0.0F){
  //         accel = maxAccel;
  //       } else {
  //         accel = -maxAccel;
  //       }
  //     }
  //     break;
  //   case MOTION_MODE_VEL:
  //     if(vel < velTarget){
  //       accel = maxAccel; 
  //     } else if (vel > velTarget){
  //       accel = -maxAccel;
  //     }
  //     break;
  // }
  // // using our chosen accel, integrate velocity from previous: 
  // vel += accel * delT;
  // // cap our vel based on maximum rates: 
  // if(vel >= maxVel){
  //   accel = 0.0F;
  //   vel = maxVel;
  // } else if(vel <= -maxVel){
  //   accel = 0.0F;
  //   vel = - maxVel;
  // }
  // // what's a position delta ? 
  // delta = vel * delT;
  // // integrate posn with delta 
  // pos += delta;
  // Serial.println(String(pos) + " " + String(vel) + " " + String(accel) + " " + String(distanceToTarget));
  // // and check in on our step modulo, 
  // stepModulo += delta;
  // if(stepModulo >= 1.0F){
  //   stepper_step(microsteps, true);
  //   stepModulo -= 1.0F;
  // } else if (stepModulo <= -1.0F){
  //   stepper_step(microsteps, false);
  //   stepModulo += 1.0F;
  // }
} // end integrator 

// ---------------------------------------------- ingest moves 
void axl_addSegmentToQueue(axlPlannedSegment_t segment){
  // get a handle for the next, 
  // and if it's marked as "ready" - it means queue is full and this is borked, 
  if(queueHead->isReady){
    // OSAP::error("on moveToQueue, looks full ahead", MEDIUM);
    return;
  }
  // we have some trust issues:
  if(segment.distance <= 0.0F){
    // OSAP::error("zero distance move", MEDIUM);
    return;
  }
  // then we just stick it in there, don't we, trusting others... 
  // since we to queueHead->isReady = true *at the end* we 
  // won't have the integrator step into this block during an interrupt... 
  // kind of awkward copy, innit? 
  // anyways these are the vals we get from the net:
  queueHead->segmentNumber = segment.segmentNumber;
  if(nextSegmentNumber != queueHead->segmentNumber){
    axl_halt(AXL_HALT_OUT_OF_ORDER_ARRIVAL);
    // haltMessage = " rx'd " + String(segment.segmentNumber) + " expected " + String(nextSegmentNumber);
  } else {
    nextSegmentNumber ++; 
  }
  // is it the last... 
  // queueHead->isLastSegment = segment.isLastSegment;
  // collect the unit vector, just all, every time, for now,
  // future implementations (which are packet-friendlier) would use dof_in_use or something
  for(uint8_t a = 0; a < AXL_MAX_DOF; a ++){
    queueHead->unit[a] = segment.unit[a];
  }
  // segment trajectory described with... 
  queueHead->distance = segment.distance;
  queueHead->vi = segment.vi; 
  queueHead->accel = segment.accel;
  queueHead->vmax = segment.vmax;
  queueHead->vf = segment.vf;
  noInterrupts();
  // and set these to allow read-out of the move
  queueHead->isReady = true;
  // and increment, 
  queueHead = queueHead->next;
  // if it isn't already: 
  mode = AXL_MODE_QUEUE;
  interrupts();
}

uint16_t axl_getSegmentAckMsg(uint8_t* msg){
  if(segmentAckMsgLen > 0){
    __disable_irq();
    memcpy(msg, segmentAckMsg, segmentAckMsgLen);
    uint16_t len = segmentAckMsgLen;
    segmentAckMsgLen = 0;
    __enable_irq();
    return len;
  } else {
    return 0;
  }
}

uint16_t axl_getSegmentCompleteMsg(uint8_t* msg){
  if(segmentCompleteMsgLen > 0){
    __disable_irq();
    memcpy(msg, segmentCompleteMsg, segmentCompleteMsgLen);
    uint16_t len = segmentCompleteMsgLen;
    segmentCompleteMsgLen = 0;
    __enable_irq();
    return len;
  } else {
    return 0;
  }
}

void axl_setPositionTarget(float _targ, float _maxVel, float _maxAccel){

}

void axl_setVelocityTarget(float _targ, float _maxAccel){

}

void axl_setPosition(float _pos){
  // trickier with queues, etc, non ? 
}

void axl_getCurrentStates(axlState_t* statePtr){
  noInterrupts();
  statePtr->pos = pos;
  statePtr->vel = vel;
  statePtr->accel = accel;
  interrupts();
}

// not yet implemented, 
void axl_halt(uint8_t haltCode){};