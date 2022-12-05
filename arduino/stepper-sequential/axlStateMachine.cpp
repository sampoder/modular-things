#include "axlStateMachine.h"
#include "stepperDriver.h"

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
volatile uint8_t queueState = AXL_QUEUESTATE_EMPTY;
volatile float pos = 0.0F;                // current position 
volatile float vel = 0.0F;                // current velocity 
volatile float accel = 0.0F;              // current acceleration 

// ---------------------------------------------- queues 
axlPlannedSegment_t queue[AXL_QUEUE_LEN];
axlPlannedSegment_t* queueHead; // ingest here, 
axlPlannedSegment_t* queueTail; // operate here, 
uint32_t queueStartDelayMS = 500;
uint32_t queueStartTime = 0;

// ---------------------------------------------- axl dof-to-motor linkage, 
uint8_t ourActuatorID = 0;

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
  // and check in on our step modulo, 
  stepModulo += delta;
  if(stepModulo >= 1.0F){
    stepper_step(microsteps, true);
    stepModulo -= 1.0F;
  } else if (stepModulo <= -1.0F){
    stepper_step(microsteps, false);
    stepModulo += 1.0F;
  }
} // end integrator 

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