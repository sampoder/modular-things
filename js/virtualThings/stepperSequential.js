/*
stepper.js
a "virtual thing" - of course 
Jake Read, Leo McElroy and Quentin Bolsee at the Center for Bits and Atoms
(c) Massachusetts Institute of Technology 2022
This work may be reproduced, modified, distributed, performed, and
displayed for any purpose, but must acknowledge the open systems assembly protocol (OSAP) and modular-things projects.
Copyright is retained and must be preserved. The work is provided as is;
no warranty is provided, and users accept all liability.
*/

import PK from "../osapjs/core/packets.js"
import { TS } from "../osapjs/core/ts.js"

export default function stepperSequential(osap, vt, name) {
  // the "vt.route" goes to our partner's "root vertex" - but we 
  // want to address relative siblings, so I use this utility:
  let routeToFirmware = PK.VC2VMRoute(vt.route)
  // here we basically write a "mirror" endpoint for each downstream thing, 
  // -------------------------------------------- 1: target data 
  // now I can index to the 1st endpoint (I know it's this one because 
  // I wrote the firmware!) just by adding a .sib() to that route;
  let targetDataEndpoint = osap.endpoint(`targetDataMirror_${name}`)
  targetDataEndpoint.addRoute(PK.route(routeToFirmware).sib(1).end())
  // -------------------------------------------- 2: motion state is a query object:
  let motionStateQuery = osap.query(PK.route(routeToFirmware).sib(2).end())
  // -------------------------------------------- 3: set current position 
  let positionSetEndpoint = osap.endpoint(`setPositionMirror_${name}`)
  positionSetEndpoint.addRoute(PK.route(routeToFirmware).sib(3).end())
  // -------------------------------------------- 4: settings, 
  let settingsEndpoint = osap.endpoint(`settingsMirror_${name}`)
  settingsEndpoint.addRoute(PK.route(routeToFirmware).sib(4).end())
  // -------------------------------------------- 5: queued moves
  let queueOutputEndpoint = osap.endpoint(`queueMirror_${name}`)
  queueOutputEndpoint.addRoute(PK.route(routeToFirmware).sib(5).end())
  // -------------------------------------------- pickup from complete moves 
  let onSegmentCompleteIn = () => {}
  let attachSegmentCompleteFn = (fn) => {
    onSegmentCompleteIn = fn 
  }
  let segmentCompleteIn = osap.endpoint(`segmentCompletes+${name}`)
  segmentCompleteIn.onData = (data) => {
    onSegmentCompleteIn(data)
  }
  // -------------------------------------------- nth: a button, not yet programmed 

  // -------------------------------------------- we need a setup, 
  const setup = async () => {
    try {
      // erp, but this firmware actually is all direct-write, nothing streams back 
      let segmentCompleteRouteUp = PK.route().sib(0).pfwd().sib(0).pfwd().sib(segmentCompleteIn.indice).end()
      // rm any existing, 
      try {
        // rm from 6 any existing routes (from previous setup)
        await osap.mvc.removeEndpointRoute(vt.children[6].route, 0)
      } catch (err) {
        // this is probably chill: the the case that the above throws an err when no-existing-route-to-rm, 
        // which is bound to happen for first time setups (!) 
      }
      // pipe 'em back up to us:
      await osap.mvc.setEndpointRoute(vt.children[6].route, segmentAckRouteUp)
    } catch (err) {
      throw err
    }
  }

  // -------------------------------------------- queue'd motion:
  let AXL_MAX_DOF = 7 
  let transmitPlannedSegment = async (unit, vi, accel, vmax, vf, dist, isLast) => {
    try {
      if(unit.length < AXL_MAX_DOF){
        for(let a = unit.length; a < AXL_MAX_DOF; a ++){
          unit[a] = 0.0;
        }
      }
      // unit, 5x floats, 1x uint32 segnum, 1x boolean isLast
      let datagram = new Uint8Array(AXL_MAX_DOF * 4 + 5 * 4 + 4 + 1)
      let wptr = 0
      // write segnum, last-ness, 
      wptr += TS.write("uint32", nextSegmentOut, datagram, wptr)
      wptr += TS.write("boolean", isLast, datagram, wptr)
      // write unit vector, 
      for(let a = 0; a < AXL_MAX_DOF; a ++){
        wptr += TS.write("float32", unit[a], datagram, wptr)
      }
      wptr += TS.write("float32", vi, datagram, wptr)
      wptr += TS.write("float32", accel, datagram, wptr)
      wptr += TS.write("float32", vmax, datagram, wptr)
      wptr += TS.write("float32", vf, datagram, wptr)
      wptr += TS.write("float32", dist, datagram, wptr)
      await queueOutputEndpoint.write(datagram, "ackless")
    } catch (err) {
      throw err 
    }
  }

  // adds new moves, doesn't flow-control input, that's application-level window 
  // EP_ONDATA_RESPONSES onQueueData(uint8_t* data, uint16_t len){
  //   // make a segment to copy-inot, 
  //   axlPlannedSegment_t segment;
  //   uint16_t rptr = 0;
  //   // location of segment-in-sequence, to count continuity, 
  //   segment.segmentNumber = ts_readUint32(data, &rptr);
  //   // is it the end of this stream ?
  //   segment.isLastSegment = ts_readBoolean(data, &rptr);
  //   // OSAP::debug("segnum, isLast " + String(segment.segmentNumber) + ", " + String(segment.isLastSegment));
  //   // unit vector describing segment's direction, in all-dof space, 
  //   for(uint8_t a = 0; a < AXL_MAX_DOF; a ++){
  //     segment.unit[a] = ts_readFloat32(data, &rptr);
  //   }
  //   // start vel, accel-rate (up, and down), max velocity, final velocity, distance (all +ve)
  //   segment.vi = ts_readFloat32(data, &rptr);
  //   segment.accel = ts_readFloat32(data, &rptr);
  //   segment.vmax = ts_readFloat32(data, &rptr);
  //   segment.vf = ts_readFloat32(data, &rptr);
  //   segment.distance = ts_readFloat32(data, &rptr);
  //   // add yonder,
  //   axl_addSegmentToQueue(segment);
  //   // that's it, but don't copy-in, 
  //   return EP_ONDATA_REJECT;
  // }

  // -------------------------------------------- Setters
  // how many steps-per-unit, 
  // this could be included in a machineSpaceToActuatorSpace transform as well, 
  let spu = 20
  // each has a max-max velocity and acceleration, which are user settings, 
  // but velocity is also abs-abs-max'd at our tick rate... 
  let absMaxVelocity = 4000 / spu
  let absMaxAccel = 10000
  let lastVel = absMaxVelocity
  let lastAccel = 100             // units / sec 

  let setPosition = async (pos) => {
    try {
      // halt... this also mode-swaps to VEL, so when we set a new posn' 
      // the motor won't slew to it 
      await stop()
      // write up a new-position-paquet, 
      let datagram = new Uint8Array(4)
      let wptr = 0
      wptr += TS.write("float32", pos, datagram, wptr)
      await positionSetEndpoint.write(datagram, "acked")
    } catch (err) {
      console.error(err)
    }
  }

  let setVelocity = async (vel) => {
    if (vel > absMaxVelocity) vel = absMaxVelocity
    lastVel = vel
  }

  let setAccel = async (accel) => {
    if (accel > absMaxAccel) accel = absMaxAccel
    lastAccel = accel
  }

  let setAbsMaxAccel = (maxAccel) => { absMaxAccel = maxAccel }

  let setAbsMaxVelocity = (maxVel) => {
    // not beyond this tick-based limit, 
    if (maxVel > 4000 / spu) {
      maxVel = 4000 / spu
    }
    absMaxVelocity = maxVel
  }

  let setCScale = async (cscale) => {
    try {
      let datagram = new Uint8Array(4)
      let wptr = 0
      wptr += TS.write("float32", cscale, datagram, wptr)  // it's 0-1, firmware checks
      // and we can shippity ship it, 
      await settingsEndpoint.write(datagram, "acked")
    } catch (err) {
      console.error(err)
    }
  }

  // tell me about your steps-per-unit, 
  // note that FW currently does 1/4 stepping: 800 steps / revolution 
  let setSPU = (_spu) => {
    spu = _spu
    if (absMaxVelocity > 4000 / spu) { absMaxVelocity = 4000 / spu }
    // we know that we have a maximum steps-per-second of 4000, so we can say 
    console.warn(`w/ spu of ${spu}, this ${name} has a new abs-max velocity ${absMaxVelocity}`)
  }

  // -------------------------------------------- Getters 

  // get states
  let getState = async () => {
    try {
      let data = await motionStateQuery.pull()
      // deserialize... 
      return {
        pos: TS.read("float32", data, 0) / spu,
        vel: TS.read("float32", data, 4) / spu,
        accel: TS.read("float32", data, 8) / spu,
      }
    } catch (err) {
      console.error(err)
    }
  }


  let getPosition = async () => {
    try {
      let state = await getState()
      return state.pos
    } catch (err) {
      console.error(err)
    }
  }

  let getVelocity = async () => {
    try {
      let state = await getState()
      return state.vel
    } catch (err) {
      console.error(err)
    }
  }

  let getAbsMaxVelocity = () => { return absMaxVelocity }
  let getAbsMaxAccel = () => { return absMaxAccel }

  // -------------------------------------------- Operative 

  // await no motion, 
  let awaitMotionEnd = async () => {
    try {
      return new Promise(async (resolve, reject) => {
        let check = () => {
          getState().then((states) => {
            console.log(`${name}\t acc ${states.accel.toFixed(4)},\t vel ${states.vel.toFixed(4)},\t pos ${states.pos.toFixed(4)}`)
            if (states.vel < 0.001 && states.vel > -0.001) {
              resolve()
            } else {
              setTimeout(check, 10)
            }
          }).catch((err) => { throw err })
        }
        check()
      })
    } catch (err) {
      console.error(err)
    }
  }

  // sets the position-target, and delivers rates, accels to use while slewing-to
  let target = async (pos, vel, accel) => {
    try {
      // modal vel-and-accels, and guards 
      vel ? lastVel = vel : vel = lastVel;
      accel ? lastAccel = accel : accel = lastAccel;
      if (accel > absMaxAccel) { accel = absMaxAccel; lastAccel = accel; }
      if (vel > absMaxVelocity) { vel = absMaxVelocity; lastVel = vel; }
      // also, warn against zero-or-negative velocities & accelerations 
      if (vel <= 0 || accel <= 0) throw new Error(`y'all are trying to go somewhere, but modal velocity or accel are negative, this won't do...`)
      // stuff a packet, 
      let datagram = new Uint8Array(13)
      let wptr = 0
      datagram[wptr++] = 0 // MOTION_MODE_POS 
      // write pos, vel, accel *every time* and convert-w-spu on the way out, 
      wptr += TS.write("float32", pos * spu, datagram, wptr)  // write posn
      wptr += TS.write("float32", vel * spu, datagram, wptr)  // write max-vel-during
      wptr += TS.write("float32", accel * spu, datagram, wptr)  // write max-accel-during
      // and we can shippity ship it, 
      await targetDataEndpoint.write(datagram, "acked")
    } catch (err) {
      console.error(err)
    }
  }

  // goto-this-posn, using optional vel, accel, and wait for machine to get there 
  let absolute = async (pos, vel, accel) => {
    try {
      // sets motion target, 
      await target(pos, vel, accel)
      // then we could do... await-move-done ? 
      await awaitMotionEnd()
      console.log(`abs move to ${pos} done`)
    } catch (err) {
      console.error(err)
    }
  } // end absolute 

  // goto-relative, also wait, 
  let relative = async (delta, vel, accel) => {
    try {
      let state = await getState()
      let pos = delta + state.pos
      // that's it my dudes, 
      await absolute(pos, vel, accel)
    } catch (err) {
      console.error(err)
    }
  }

  // goto-this-speed, using optional accel, 
  let velocity = async (vel, accel) => {
    try {
      // modal accel, and guards... 
      accel ? lastAccel = accel : accel = lastAccel;
      if (accel > absMaxAccel) { accel = absMaxAccel; lastAccel = accel; }
      if (vel > absMaxVelocity) { vel = absMaxVelocity; lastVel = vel; }
      // note that we are *not* setting last-vel w/r/t this velocity... esp. since we often call this 
      // w/ zero-vel, to stop... 
      // now write the paquet, 
      let datagram = new Uint8Array(9)
      let wptr = 0
      datagram[wptr++] = 1 // MOTION_MODE_VEL 
      wptr += TS.write("float32", vel * spu, datagram, wptr)  // write max-vel-during
      wptr += TS.write("float32", accel * spu, datagram, wptr)  // write max-accel-during
      // mkheeeey
      await targetDataEndpoint.write(datagram, "acked")
    } catch (err) {
      console.error(err)
    }
  }

  // stop !
  let stop = async () => {
    try {
      await velocity(0)
      await awaitMotionEnd()
    } catch (err) {
      console.error(err)
    }
  }

  // we return fns that user can call, 
  return {
    transmitPlannedSegment,
    attachSegmentCompleteFn,
    // operate w/
    // target,
    // absolute,
    // relative,
    // velocity,
    // stop,
    // awaitMotionEnd,
    // // setters... 
    // setPosition,
    // setVelocity,
    // setAccel,
    // setAbsMaxAccel,
    // setAbsMaxVelocity,
    // setCScale,
    // setSPU,
    // // inspect... 
    // getPosition,
    // getVelocity,
    // getAbsMaxVelocity,
    // getAbsMaxAccel,
    // these are hidden 
    setup,
    vt,
    // api: [
    //   {
    //     name: "absolute",
    //     args: [
    //       "pos: [x, y, z]",
    //     ]
    //   },
    //   {
    //     name: "relative",
    //     args: [
    //       "pos: [x, y, z]",
    //     ]
    //   },
    //   {
    //     name: "setVelocity",
    //     args: [
    //       "number",
    //     ]
    //   },
    //   {
    //     name: "setAccel",
    //     args: [
    //       "number",
    //     ]
    //   },
    //   {
    //     name: "stop",
    //     args: []
    //   },
    //   {
    //     name: "awaitMotionEnd",
    //     args: []
    //   },
    //   {
    //     name: "getState",
    //     args: [],
    //     return: `
    //       { 
    //         pos: [x, y, z], 
    //         vel: number, 
    //         accel: number 
    //       }
    //     `
    //   },
    //   {
    //     name: "getAbsMaxVelocity",
    //     args: [],
    //     return: "number",
    //   },
    //   {
    //     name: "setCScale",
    //     args: [
    //       "number",
    //     ]
    //   },
    //   {
    //     name: "setSPU",
    //     args: [
    //       "number",
    //     ]
    //   }
    // ]
  }
}