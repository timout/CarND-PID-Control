# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Objective
This project is to use PID controllers to control the steering angle and the throttle for driving a car in a game simulator. The simulator provides cross-track error (CTE) via websocket. The PID (proportional-integral-differential) controllers give steering and throttle commands to drive the car reliably around the simulator track.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Parameters of PID controller
Description of PID values in PID control  

* P (proportional) accounts for present values of the error. 
* I (integral) accounts for all past values of the error.  
* D (differential) accounts for possible future trends of the error, based on its current rate of change.

PID parameters used for steering angles:

* P = 0.195138, -7.286e-13, 
* I = value: -7.286e-13
* D = value: 2.9994

## Parameter tuning

I have implemented 2 algorithms:  
* Gradient Descent with Backpropagation
* Twiddle

## Thoughts and improvents
* Tuning with Twiddle takes more time so it was implemented limited number of iterations. Reset every `epoch_size` iterations. Than need some improvement, probably any way to avoid it.
* SG implementation depends on initial parameters so with poor choice the car often goes out of the road. So reset function was implemented and requires some improvement.
* Only steering controller was implemented. Next step will speed contoller.

## Result

Result video:  
![Video](out.gif?raw=true)  