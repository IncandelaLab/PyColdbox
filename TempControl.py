# Modules

import time
import pyserial
import instrumentino
import numpy

# PID Control

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
 PID.SetPoint=input("Temperature Setpoint: ")
 PID.Kp=input("Proportional Constant: ")
 PID.Ki=input("Integral Constant: ")
 PID.Kd=input("Derivative Constant: ")
 
 coolport=input("cooling box port: ")
 coolrate=input("cool box rate: ")

#serial port config
 """gray cable serial port: polarity changer"""
 """arduino serial port"""
 arduino=serial.Serial('serial name, rate')
 polaritybox=serial.Serial(coolport, float(coolrate))

#reading tempterature and humidity
  #some code to read temp and humidity here that needs to access the arduino
  #TBD some kind of iteration scheme that averages temperature
  #this is adapted from the LabVIEW code right now so it's jank

humlist=numpy.array([0,0])
templist=numpy.array([0,0,0,0])

while """some condition for looping to read temperature/iteration number""":

    temp1=arduino.read('read temp from the arduino')
    temp2=arduino.read('read temp from the arduino')
    temp3=arduino.read('read temp from the arduino')
    temp4=arduino.read('read temp from the arduino')
    
    templist=templist+numpy.array([temp1, temp2, temp3, temp4])

    hum1=arduino.read('read humidity from the arduino')
    hum2=arduino.read('read humidity from the arduino')

    humlist=humlist+numpy.array([hum1,hum2])

avgtemplist=templist/'iteration number'
    
    #for pid control
lasttemp=sum(avgtemplist)/len(templist) #average of all temperatures for PID control to use
temperror=PID.setpoint-lasttemp 

    #changing the polarity
while temperror <= 0:
    polaritybox.write('co') #changes to cool
while temperror > 0:
    polaritybox.write('ch') #changes to heat
if temperror=('within +-2 deg of setpoint'):
    'stop doing things'



#probably some gui stuff in here
#instrumentino seems ideal but like... whatever


 


