## 2022 11 21 

### Modular-Thing Motion Systems 

I think that for this example, each motor should be 1DOF, since we can / will directly address each... and we can do basics:

- each has abs. maximum acceleration and rate, 
- each has a state machine... w/ velocity or position targets, 
- we have two "moves"
  - goto posn w/ <accel>, <vf>, <vi>
  - goto velocity w/ <accel> 
  - in either, we can leave off args at the tail (shorter length packets)
    - in which case i.e. accel uses abs-max, and (vf, vi) go to (zero, current)
- we have *one* "query-ables"
  - motion state: position, velocity, acceleration 
  - if we're moving, vel != 0, that's .awaitMotionEnd() 
- we can set some things:
  - set abs. maximums 
  - set current position 

This seems to me like the minimum viable, on top of which everything else can be written... and homing, etc, we'll do with external switches. 

### 2022 11 22

I think I should start with outfitting a little integrator, using above states, and get it into a QTPY, w/o an interrupt to start, to look at floating-point calculation times. Also needs some basis in the axl codes, i.e. I need to look at them. 

Hmm... OK floating point ops don't look too eggregious yet, though I've just a few multiplications. I am seeing a deal of clock drift though, so I will want to check in on that front as well... and might end up wanting some kind of time-sync algo ?? 

I'm getting a sense that this might actually work, though am a little confused as to how - I think I might check in on clock drift now, ahead of finishing the full blown state machine, since it's not unlikely that error there could pooch the whole situation. 

Clock sync actually looks OK, god bless. 

So, now I want a full on state machine in here, and to outfit it, then demo with... a step / dir pin set on the scope (first, just going to a pre-set position), then w/ a virtual machine - since i.e. we are going to live in straight one-step-per-unit land down under, yeah? 

OK, it's apparently working an instrumented, but needs hardware output (probably next), then a VM, and interrupts. 

I'll actually swap on now to see that I can get the hardware working: so a coupla debug-outs here, then PWM waveforms and hopefully sinusoids on the other side... I'll be copy-pasta from the fab-step project most likely. 

This works now as well, thing still isn't on an interrupt though, I suppose that would be next: then the vm / etc. 

## 2022 11 28 

OK and today we'll do this machine API.

```js
const machine = createMachine(
  [motor1, motor2, motor3],
  (targetCoordinates) => { return transformedCoords },
  (motorCoordinates) => { return targetCoordinates }
)

machine.setMaxAccel(accel)
machine.setVelocity(rate)
machine.absolute([x,y,z], rate = last, accel = last)
machine.relative([x,y,z], rate = last, accel = last)
machine.setPosition([x,y,z])
machine.stop()
```

OK, ish... though perhaps .absolute, .relative should use ([x,y,z]) rather than "raw args" - easier later to disambiguate from other args i.e. if we have (targ, rate, accel)

... it's troublesome, this layer, as it's most appropriately user-code, methinks. position transforms != velocity transforms, etc... hidden modal state, etc... 

But, I'll wrap on this soon, and then I think the most productive thing would be to get a demo wrippen: so, motor mount hardware, ahn machine... limits... or I could do circuit assembly. 

OK, that's all that, I'm going to get into harware now... 

## 2022 11 29

Working on this again, last night we thought to call 'em "synchronizers" and do transforms elsewhere. I think I'll do this, then see about writing the transforms, etc, for a corexy machine. 

- setting maximums:
  - motor.setVelocity, motor.setAcceleration 
  - sync.setVelocities, sync.setAccelerations (?) 
- using 'em 
  - motor.absolute, motor.relative, motor.velocity 
  - sync.absolute, sync.relative, sync.velocity 
- refactor sync factory thing to match motor-set... 

It's... comin, I have realized that each motor *needs* abs-max velocities *as well as* abs-max accels, then also modal-settings for each: what to use. Means a lot of functions to access all of this... but the consistency is nice, at least. 

When I get back, then,

- absMax accel, vel in the motor 
- check against both... whenever we might need to, right ? 
  - use .target() as the base, call that elsewhere
  - .velocity() is the "other base" - innit ? 
- test with... pots-to-targets ? 

Sheesh: OK, I think it's, like, done, and I want to try wiring it to a machine, and try also the position-target-setting-on-the-fly. 

OK - I have the circles-thing up again, with... a new API? IDK - I'm going to try next a floating-target example, with a potentiometer maybe ~ or something, lol, then call it and start doing the organizing, etc. 

## 2022 12 04 

OK I wanted to see if I could whip sequential motion into this thing... I think it means ~ copypasta some from the axl project, but simpler piping, but still some wiring up / down etc. 

I think, since this is ~ something of a pairing-down exercise, I'll try a simple motor code that *just* does sequences, then we can see about merging them togther later on. 

Let's see: 

- we send pre-calculated segments 
- we get an ack when they're ingested (flow-controlled)
- we get a "done" when they're done moving 
- we do windowed: when we get "done" from seg `n`, we are txing seg `n + queue_len` -> OK 

That's kind of it, innit? So we need to tx segments down, which should be ~ vi, accel, vmax, dist (delta), vf, that's it. The awkward things are, like, rx'ing segments that are not-along-our-axis... so we might need a better segment rep, i.e. one that includes like `segment_execution_time` ??? 

The current AXL implementation handles this by sending the whole gd axis count down... other implementations use step-based representations, like `steps_to_accel, steps_to_cruise, steps_until_decel`, then just state-machine their way thru moves, doing the nasty calculations up front. 

I ~ kind of like the current AXL implementation, at least until we get up to ~ the ideal spline-implementation. Segments, everyone steps thru 'em together, etc. It means sucking down 6-DOF (as a fixed count) of motion in each segment, though, so it'll bake through RAM. The E18A has only 32kb of RAM, so let's see... if a move has 5x floats (vi, vf, vmax, accel, dist), then 6x dof, we have 30 * 4 = 120 bytes per segment (at least), then for ~ 32 segments we suck 4k ram into the queue alone... it's a bit greedy, innit, but it might be the move for the time being, since we know it to work pretty well in AXL? 

And - wait - it isn't that eggregious: we have NDOF x 1 for the unit vector, then 1x each other. So this is actually a totally sane way to do this, and I think I will do, thank you very much. So I can do 7 DOF (for a robot arm +1, idk lol) - seems like more than we would ever need, future folks can go ++ if they'd like. 

## 2022 12 05 

OK it's time-oclock to actually implement this... being a little more flash-savvy now. I guess on the outer loop I should get my plumbing together, then the queueing system itself (?), then setup a test... thing, I guess in modular-thing, shackles and all. 

Next... I'm ending up ~ round-about implementing everything here, from axl, but I am maybe about to run out of flash, so might have to stack-acks in a single message-return endpoint... 

- instrument return pipes 
- needs to select ourAxisID
- ... actually run the thing ? 
- js piping / setup ? 
- recall that js has ackless transmit of segments, since acks are piped back via setup-route, 

---

- later
  - do split-screen modular-thing demos 
  - do .setSPU, .setCSCale -> .setStepsPerUnit, .setCurrent
  - do xylophone demo 