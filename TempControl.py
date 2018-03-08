# Modules

import time
import pyserial
import numpy

# PID Control
#this code is from ivPID (may leave in its own file and call from here instead?)

class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

#User Inputs
 PID=PID.PID(P,I,D)
 PID.SetPoint=input("Temperature Setpoint: ")
 PID.Kp=input("Proportional Constant: ")
 PID.Ki=input("Integral Constant: ")
 PID.Kd=input("Derivative Constant: ")
    #ports
 coolport=input("cooling box port: ")
 coolrate=input("cool box rate: ")

 ardport=input("arduino port: ")
 ardrate=input("arduino rate: ")

#functions to make this reasonable
convert= lambda x: float(str(x)) 

#serial port config
 """gray cable serial port: polarity changer"""
 """arduino serial port"""
 arduino=serial.Serial(ardport, ardrate)
 polaritybox=serial.Serial(coolport, coolrate)

#for pid control
while '''condition for reading data''':
	arduino.write('AVG\r')
	output=arduino.readline()
    tempavg=convert(output[2,-5]) #why do I have to turn this into a string first. curse you and your b character

    feedback_value=tempavg #the pid will use this value to calculate error

#changing the polarity (heat/cool) appropriately
while error <= -2.0:
    polaritybox.write('co') #changes to cool
while error > 2.0:
    polaritybox.write('ch') #changes to heat
else:
    polaritybox.write('cc') #stops cooling

#duty cycle is a number between 0 and 0.5
#temp will go between 80F and like... -30C? remember to ask about this
#detector operates down to ~-20 max
#turn the PID into a number between 0 and 0.5 we can use
#110.0 is arbitrarily chosen for the estimated range of the coldbox
dutycyc=PID/110.0
if dutycyc > 0.5:
 	print("duty cycle value above 0.5! not allowed!") #until we figure out how to translate the PID properly
else:
    #note to self: add an arduino function that writes the duty cycle to its appropriate pin. Call it DUTY
    arduino.write('DUTY'+str(dutycyc)+'\r') #command name + value + escape character







 


