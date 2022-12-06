/*
synchronizer.js

a "virtual machine" - of course 

Jake Read, Leo McElroy and Quentin Bolsee at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2022

This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the open systems assembly protocol (OSAP) and modular-things projects.
Copyright is retained and must be preserved. The work is provided as is;
no warranty is provided, and users accept all liability.
*/

/*
const machine = createSynchronizer(
  [motor1, motor2, motor3],
  (targetCoordinates) => { return transformedCoords },
  (motorCoordinates) => { return targetCoordinates }
)

machine.setMaxAccel(accel)
machine.setMaxVelocity(rate)
machine.absolute([x,y,z], rate = last, accel = last)
machine.relative([x,y,z], rate = last, accel = last)
machine.setPosition([x,y,z])
machine.stop()

// my thoughts / modifications:
- am I doing the factory correctly here? I would return an object, rather than the machine.fn everywhere... 
- we can do without .setMaxAccel and .setMaxVelocity, those are user-impositions, 
  - rather use .absolute and .relative to have (..., rate, accel) arguments 
  - and let those args be modal: if they aren't supplied, use the most-recently-used, 
- how do we throw / catch errors, since machines call motors ? 
  - can we do a higher-level wrap so that we can throw 'em always all the way up to user code ? 
- also... .setPosition / etc, is a function of the transform, innit ? 
- transforms for *position* are not identical to *velocity* transforms, 
  - if the transform is linear, we should be able to just use a delta transform... 
  - nonlinear (i.e. angular) transforms leave us fully beached for velocities, etc (?) 
- this is complex in surprising ways... 
*/

import TIME from "../osapjs/core/time.js"
import { TS } from "../osapjs/core/ts.js"

// these are middling vector-maths implementations, my apologies

// addition... 
let vectorAddition = (A, B) => {
  return A.map((a, i) => { return A[i] + B[i] })
}

// distances from a-to-b, 
let vectorDeltas = (A, B) => {
  return A.map((a, i) => { return B[i] - A[i] })
}

// between A and B 
let vectorDistance = (A, B) => {
  let numDof = A.length
  let sum = 0
  for (let a = 0; a < numDof; a++) {
    sum += Math.pow((A[a] - B[a]), 2)
  }
  return Math.sqrt(sum)
}

// from A to B 
let unitVector = (A, B) => {
  let numDof = A.length
  let dist = vectorDistance(A, B)
  let unit = new Array(numDof)
  for (let a = 0; a < numDof; a++) {
    unit[a] = (B[a] - A[a]) / dist
  }
  return unit
}

// of A 
let unitOf = (A) => {
  let numDof = A.length 
  let dist = vectorDistance(A, (new Array(numDof)).fill(0))
  let unit = new Array(numDof)
  for (let a = 0; a < numDof; a++) {
    unit[a] = (A[a]) / dist
  }
  return unit
}

export default function createSynchronizer(actuators) {
  if (!Array.isArray(actuators)) throw new Error(`pls, an array of actuators`)
  // some state... our most-recently used accel & velocity, 
  // there's going to be a little bit of trouble w/r/t motor absolute-max-velocities 
  // and what the machine. requests of them, 
  let lastAccel = 100
  let lastVel = 100
  // sometimes we know this, and that can speed things up, other times we are unawares 
  let lastAbsolute = null

  // -------------------------------------------- Queues... 
  let segComplete = (a, num) => {
    // will pop from list here, check against all-else, etc... 
    console.warn(`synchronizer rx complete from ${a}, num ${num}`)
    // find and pop from queue, 
    for(let s in queue){
      for(let c in queue[s].actuators){
        console.log(c, a, c == a, queue[s].segmentNumber == num)
      }
      // mark 'em rx'd, then re-check queue states 
    }
  }
  for(let a = 0; a < actuators.length; a ++){
    actuators[a].attachSegmentCompleteFn((data) => {
      let segNum = TS.read("uint32", data, 0)
      segComplete(a, segNum)
    })
  }
  let queue = [] 
  let jsQueueStartDelay = 500 
  let QUEUE_STATE_EMPTY = 1
  let QUEUE_STATE_AWAITING_START = 2
  let QUEUE_STATE_RUNNING = 3
  let QUEUE_STATE_HALTED = 4
  let AXL_MAX_DOF = 7
  let AXL_REMOTE_QUEUE_LENGTH = 30 // actual in embedded is 32, use this to start... 
  let queueState = QUEUE_STATE_EMPTY
  let nextSegmentOut = 0 
  let lastTargetPosition = null 
  let addMoveToQueue = async (target, vel, accel) => {
    try {
      // modal vel-and-accels,
      vel ? lastVel = vel : vel = lastVel;
      accel ? lastAccel = accel : accel = lastAccel;
      // arrays only 
      if(!Array.isArray(target)) throw new Error(`arrays only into <target> for addMoveToQueue pls`)
      // we use a stateful last-posn to reckon deltas, 
      // this is stored in axl's full width... 
      if(!lastTargetPosition){
        lastTargetPosition = (new Array(AXL_MAX_DOF)).fill(0)
        console.warn(`on queue startup, init last-posn array to`, lastTargetPosition)
      }
      // fill the target to match DOF count 
      if(target.length < AXL_MAX_DOF) target = target.concat((new Array(AXL_MAX_DOF - target.length)).fill(0))
      // console.warn(`filled target, is now `, target)
      // get the delta move:
      let deltas = vectorDeltas(lastTargetPosition, target)
      // console.warn(`deltas`, deltas)
      // and the distance,
      let dist = vectorDistance(lastTargetPosition, target)
      // console.warn(`dist`, dist)
      // and the unit of that,
      let unit = unitOf(deltas)
      console.warn(`unit`, unit[0].toFixed(3))
      // OK, we should check 'em against max accels / max velocities, 
      // we're also going to need to know about each motor's abs-max velocities:
      let absMaxVelocities = actuators.map(actu => actu.getAbsMaxVelocity())
      let absMaxAccels = actuators.map(actu => actu.getAbsMaxAccel())
      // these are our candidate vels & accels for the move, 
      let velocities = unit.map((u, i) => { return Math.abs(unit[i] * vel) })
      let accels = unit.map((u, i) => { return Math.abs(unit[i] * accel) })
      // but some vels or accels might be too large, check thru and assign the biggest-squish to everything, 
      let scaleFactor = 1.0
      for (let a in actuators) {
        if (velocities[a] > absMaxVelocities[a]) {
          let candidateScale = absMaxVelocities[a] / velocities[a]
          if (candidateScale < scaleFactor) scaleFactor = candidateScale;
        }
        if (accels[a] > absMaxAccels[a]) {
          let candidateScale = absMaxAccels[a] / accels[a]
          if (candidateScale < scaleFactor) scaleFactor = candidateScale;
        }
      }
      // we want to apply that factor to the move's vel & accel, 
      // though - this a bug - we could have different max-vel scaling and max-accel scaling, non ?
      vel = vel * scaleFactor;
      accel = accel * scaleFactor;
      // OK so we have a legal move now: a unit vector, vmax (vel), accel, 
      // but no vi, vf. 
      // time being, let's do this:
      let vi = vel * 0.25 
      let vf = vel * 0.25 
      // yep, lol, now let's queue em, right ? 
      // I think that means... 
      // - stuff into an object
      // - stuff into the queue (w/ time-released, etc info)
      // - do checkQueueStates, 
      let segment = {
        unit, vi, accel, vmax: vel, vf, dist,
        segmentNumber: nextSegmentOut,
        // isLastSegment: false, 
        transmitTime: 0,
      }
      console.warn(`adding segment: \nvi: ${vi.toFixed(2)}\t vmax: ${vel.toFixed(2)}\t vf: ${vf.toFixed(2)}\naccel: ${accel.toFixed(2)}\t dist: ${dist.toFixed(2)}\nnum: ${nextSegmentOut}`)
      nextSegmentOut ++ 
      lastTargetPosition = target 
      // add to le queueue
      queue.push(segment)
      // here is where we could do runQueueOptimization()
      // checkQueueState looks at tx states, etc, and tx's, etc 
      await checkQueueState()
    } catch (err) {
      throw err 
    }
  }

  let checkQueueState = async () => {
    try {
      switch (queueState) {
        case QUEUE_STATE_EMPTY:
          if (queue.length > 0) {
            queueState = QUEUE_STATE_AWAITING_START
            setTimeout(() => {
              console.warn(`QUEUE START FROM AWAITING...`)
              queueState = QUEUE_STATE_RUNNING
              checkQueueState()
            }, jsQueueStartDelay)
          }
          break;
        case QUEUE_STATE_AWAITING_START:
          // noop, wait for timer... 
          break;
        case QUEUE_STATE_RUNNING:
          // can we publish, do we have unplanned, etc?
          // console.warn(`QUEUE RUNNING...`)
          // so we'll try to transmit up to 32 ? and just stuff 'em unapologetically into the buffer, leggo: 
          for (let m = 0; m < AXL_REMOTE_QUEUE_LENGTH - 1; m++) {
            if (!queue[m]) {
              // console.warn(`breaking because not-even-32-items here...`)
              break;
            }
            if (queue[m].transmitTime == 0) {
              // console.log(`tx'd item at ${m}, segment ${queue[m].segmentNumber}`)
              await transmitSegment(queue[m])
            }
          }
          break;
        case QUEUE_STATE_HALTED:
          console.warn(`halted, exiting...`)
          break;
        default:
          console.error(`unknown state...`)
          break;
      } // end switch 
    } catch (err) {
      throw err
    }
  }

  let transmitSegment = async (segment) => {
    try {
      segment.transmitTime = TIME.getTimeStamp()
      // we have to track acks-from-actuators, 
      segment.actuators = []
      await Promise.all(actuators.map((actu, i) => { 
        segment.actuators.push(i)
        return actu.transmitPlannedSegment(segment)
      }))
    } catch (err) {
      throw err 
    }
  }

  // -------------------------------------------- Setters 

  let setPosition = async (pos) => {
    try {
      // we... should guard against same-sizeness as well, yikes 
      if (!Array.isArray(pos)) throw new Error(`pls, an array of posns, to set`)
      await Promise.all(actuators.map((actu, i) => { return actu.setPosition(pos[i]) }))
    } catch (err) {
      console.error(err)
    }
  }

  // set... the speed we'd like to travel, in straight lines... 
  let setVelocity = (vel) => { lastVel = vel }

  // and the accel to use, 
  let setAccel = (accel) => { lastAccel = accel }

  // -------------------------------------------- Getters 

  let getPosition = async () => {
    try {
      let posns = await Promise.all(actuators.map(actu => actu.getPosition()))
      return posns
    } catch (err) {
      console.error(err)
    }
  }

  let getVelocity = async () => {
    try {
      let vels = await Promise.all(actuators.map(actu => actu.getVelocity()))
      return vels
    } catch (err) {
      console.error(err)
    }
  }

  // -------------------------------------------- Operative 

  let awaitMotionEnd = async () => {
    try {
      // just await all stop, 
      await Promise.all(actuators.map(actu => actu.awaitMotionEnd()))
    } catch (err) {
      console.error(err)
    }
  }

  // todo... just aim at it, don't await end, 
  let target = async (pos, vels, accels) => {
    try {
      // set all downstream... 
      // we can't really do 'sync' stuff for a target, since we are asking each motor to slew from wherever it is to this new posn, 
      // but we could optionally pass in arrays of vels / accels for the motors to use, 
      await Promise.all(actuators.map((actu, i) => { return actu.target(pos[i], vels ? vels[i] : undefined, accels ? accels[i] : undefined) }))
      // can't know this anymore, 
      lastAbsolute = null
    } catch (err) {
      console.error(err)
    }
  }

  // goto this absolute actuator-position 
  let absolute = async (pos, vel, accel) => {
    try {
      // modal vel-and-accels,
      vel ? lastVel = vel : vel = lastVel;
      accel ? lastAccel = accel : accel = lastAccel;
      // if we don't know the lastest machine position, grab it... 
      if (!lastAbsolute) lastAbsolute = await getPosition()
      // where we're going... 
      let nextAbsolute = pos
      // we're also going to need to know about each motor's abs-max velocities:
      let absMaxVelocities = actuators.map(actu => actu.getAbsMaxVelocity())
      let absMaxAccels = actuators.map(actu => actu.getAbsMaxAccel())
      // and a unit vector... I know this should be explicit unitize-an-existing-vector, alas, 
      let unit = unitVector(lastAbsolute, nextAbsolute)
      // these are our candidate vels & accels for the move, 
      let velocities = unit.map((u, i) => { return Math.abs(unit[i] * vel) })
      let accels = unit.map((u, i) => { return Math.abs(unit[i] * accel) })
      // but some vels or accels might be too large, check thru and assign the biggest-squish to everything, 
      let scaleFactor = 1.0
      for (let a in actuators) {
        if (velocities[a] > absMaxVelocities[a]) {
          let candidateScale = absMaxVelocities[a] / velocities[a]
          if (candidateScale < scaleFactor) scaleFactor = candidateScale;
        }
        if (accels[a] > absMaxAccels[a]) {
          let candidateScale = absMaxAccels[a] / accels[a]
          if (candidateScale < scaleFactor) scaleFactor = candidateScale;
        }
      }
      // apply that factor to *both* vels and accels, 
      velocities = velocities.map(v => v * scaleFactor)
      accels = accels.map(a => a * scaleFactor)
      // ok, sheesh, I think we can write 'em, do this with promise.all so that 
      // each message dispatches ~ at the same time, thusly arriving ~ at the same time, to get-sync'd 
      await Promise.all(actuators.map((actu, i) => {
        return actu.absolute(nextAbsolute[i], velocities[i], accels[i])
      }))
      // motors each await-motion-end, when we await-all .absolute, so by this point we have made the move... can do 
      lastAbsolute = pos
    } catch (err) {
      console.error(err)
    }
  }

  // move relative... 
  let relative = async (deltas, vel, accel) => {
    try {
      // if we don't know the lastest machine position, grab it... 
      if (!lastAbsolute) lastAbsolute = await getPosition()
      // and just... do... 
      let nextAbsolute = vectorAddition(lastAbsolute, deltas)
      await absolute(nextAbsolute, vel, accel)
    } catch (err) {
      console.error(err)
    }
  }

  // we want the motor to go along this velocity vector, probably with matched acceleration, 
  let velocity = async (vels, accel) => {
    try {
      if (!Array.isArray(vels)) throw new Error(`pls, a velocity vector here`)
      accel ? lastAccel = accel : accel = lastAccel;
      // so we need to collect the motors' absolute max accels, 
      let absMaxAccels = actuators.map(actu => actu.getAbsMaxAccel())
      // get a unito, 
      let unit = unitVector(vels)
      // as above, so here (below), we need to check accels, vels, against possible... 
      let accels = unit.map((u, i) => { return Math.abs(unit[i] * accel) })
      let velocities = vels // erp, 
      // might be toooo bigly, 
      let scaleFactor = 1.0 
      for(let a in actuators){
        if (velocities[a] > absMaxVelocities[a]) {
          let candidateScale = absMaxVelocities[a] / velocities[a]
          if (candidateScale < scaleFactor) scaleFactor = candidateScale;
        }
        if (accels[a] > absMaxAccels[a]) {
          let candidateScale = absMaxAccels[a] / accels[a]
          if (candidateScale < scaleFactor) scaleFactor = candidateScale;
        }
      }
      // apply that factor to *both* vels and accels, 
      velocities = velocities.map(v => v * scaleFactor)
      accels = accels.map(a => a * scaleFactor)
      // ok, we have a set of accels, now we can do like... 
      await Promise.all(actuators.map((actu, i) => {
        return actu.velocity(velocities[i], accels[i])
      }))
    } catch (err) {
      console.error(err)
    }
  }

  // halt... 
  let stop = async () => {
    try {
      // (1) set velocity-targets across all to zero, 
      await Promise.all(actuators.map(actu => actu.stop()))
      // (2) collect new position, given that some unknown amount of decelleration occured 
      lastAbsolute = await getPosition()
    } catch (err) {
      console.error(err)
    }
  }

  return {
    // listicle, 
    actuators,
    // sync'd motion 
    addMoveToQueue,
    // operate w/ 
    target,
    absolute,
    relative,
    velocity,
    stop,
    awaitMotionEnd,
    // setters
    setPosition,
    setVelocity,
    setAccel,
    // getters, 
    getPosition,
    getVelocity,
  }
}