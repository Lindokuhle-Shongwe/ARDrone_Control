import time
import math
from decimal import *
from pyardrone import ARDrone,at
from contextlib import suppress
import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev
import struct
import sys

#GLOBAL VARIABLES
#-------------------------------------------------------------------------------------------------
pipes = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1], [0xF0, 0xF0, 0xF0, 0xF0, 0xE1]]
radio = NRF24(GPIO, spidev.SpiDev())
sample_time=0.05
u=[0,0,0]
e=[0,0,0]
e1=[0,0,0]
u1=[0,0,0]
pen=[0,0]
ref=0;
pitch_ref=0
roll_ref=0
x=[0,0]
y=[0,0]

x_ang=[0,0,0,0,0,0]
y_ang=[0,0,0,0,0,0]
phi=  [0,0,0,0,0,0]
theta=[0,0,0,0,0,0]

#-------------------------------------------------------------------------------------------------

#GLOBAL FUNCTIONS
#-------------------------------------------------------------------------------------------------
#initialise drone object
def init_drone():
    #in this block of code we set the altitude, euler, and vertical velocity limit of the drone
    drone.navdata_ready.wait()
    drone.send(at.CONFIG('general:navdata_demo',True))  #request navdata
    drone.send(at.CONFIG('control:euler_angle_max',0.52)) #set to maximum value of 30 degrees
    drone.send(at.CONFIG('control:altitude_max',6000)) #set max altitude
    drone.send(at.CONFIG('control:control_vz_max',2000)) #set max vertical velocity
    drone.send(at.FTRIM()) #calibrate sensors
    time.sleep(3)
#initialise the NRF24L01 radio object
def init_radio():
    GPIO.setmode(GPIO.BCM)
    radio.begin(0, 17)
    radio.setPayloadSize(15)
    radio.setChannel(0x76)
    radio.setDataRate(NRF24.BR_1MBPS)
    radio.setPALevel(NRF24.PA_MIN)
    radio.setAutoAck(True)
    radio.enableDynamicPayloads()
    radio.enableAckPayload()
    radio.openReadingPipe(1, pipes[1])
    radio.printDetails()
    radio.startListening()

#Read data which was transmitted to the NRF
def read():
    receivedMessage = []
    radio.read(receivedMessage, radio.getDynamicPayloadSize())
    #print("Received: {}".format(receivedMessage))
    string =""
    for n in receivedMessage:
#     Decode into standard unicode set 
       if (n >= 32 and n <= 126):
        string += chr(n)   
    return string

#create new drone object and initialize

drone = ARDrone()
init_drone()
file=open("results.txt","w")
#file=open("vibration.txt","w")
file.write("theta phi u[0] e[0] \n")
#file.write("x_angle y_angle phi theta")

#initialise and read from radio object
init_radio()
ready=1

while(ready):
    try:
        pendulum_state=read()
        [a,b]=pendulum_state.split()
        ready=0
    except:
        print("No pendulum data received")
        ready=1
count=0
quad=0

drone.takeoff()
time.sleep(10)

while(count<1000):
    x[1]=x[0]
    y[1]=x[0]
    try:
        pendulum_state=read()
        [x[0],y[0]]=pendulum_state.split()
        
    except:
        #in the case that a data packet is lost in transmission
        x[0]=x[1]
        y[0]=y[1]
        
    #convert angle measured in body frame from milliradians to radians and apply correction
    y_angle=round(-float(x[0])/1000,2);
    x_angle=round(-float(y[0])/1000,2);

    offset=0.05
    if (x_angle>=offset and y_angle>=offset):
        quad=1
    elif (y_angle>=offset and x_angle<-offset):
        quad=2
    elif (y_angle<-offset and x_angle<-offset):
        quad=3
    elif (y_angle<-offset and x_angle>=offset):
        quad=4
    else:
        quad=0
    
    #in this section we calculate the pendulum angle using small angle approximation
    ang_x=(math.pi/2)-math.fabs(x_angle)
    ang_y=(math.pi/2)-math.fabs(y_angle)

    #crunch through all the geometry
    c=(math.pow(0.9,2)*math.pow(math.tan(ang_y),2))
    d=math.pow(math.tan(ang_y),2)+math.pow(math.sin(ang_x),2)
    l1_2=c/d
    l1=math.sqrt(c/d)
    l2_2=(c/d)*math.pow((math.sin(ang_x)/math.sin(ang_y)),2)
    l2=math.sqrt(l2_2)
    l3_2=l1_2*math.pow(math.cos(ang_x),2)+l2_2*math.pow(math.cos(ang_y),2)
    l3=math.sqrt(l3_2)
    c=l1*math.sin(ang_x)
     
    # calculate the angle of the pendulum
    pen[1]=pen[0]
    pen[0]=round(math.acos(c/0.9),1)
    pen_vel=(pen[0]-pen[1])/0.05
    
    #compute control action (u)
    u[2]=round(u[1],4)
    u[1]=round(u[0],4)
    e[2]=round(e[1],4)
    e[1]=round(e[0],4)
    e[0]=round(ref-pen_vel,4)
    u[0]=u[1]*1.9512-0.9512*u[2]+e[2]*(0.000007959)
    u[0]=round(u[0],4)
    
    #calculate plant control action (u2)
    u1[2]=round(u1[1],4)
    u1[1]=round(u1[0],4)
    e1[2]=round(e1[1],4)
    e1[1]=round(e1[0],4)
    e1[0]=round(u[0]-pen_vel,34)
    u1[0]=1.6285*u1[1]-0.6285*u1[2]-9.0543*(e1[0]-1.688*e[1]+0.7154*e[2])
    u1[0]=round(u1[0],4)
    
    #convert control action to component accelerations
    acc_y=u1[0]*l2*math.cos(ang_y)/l3;
    acc_x=u1[0]*l1*math.cos(ang_x)/l3;
    
    #convert component accelerations to pitch/roll references
    if (quad==1):
        pitch_ref=math.atan(acc_x/9.8)/1.3
        roll_ref=math.atan(acc_y/9.8)/1.3
    elif (quad==2):
        pitch_ref=-math.atan(acc_x/9.8)/1.3
        roll_ref=math.atan(acc_y/9.8)/1.3
    elif (quad==3):
        pitch_ref=-math.atan(acc_x/9.8)/1.3
        roll_ref=-math.atan(acc_y/9.8)/1.3
    elif (quad==4):
        pitch_ref=math.atan(acc_x/9.8)/1.3
        roll_ref=-math.atan(acc_y/9.8)/1.3
    else:
        pitch_ref=0
        roll_ref=0
    
    pitch_ref= round(pitch_ref,6)
    roll_ref= round(roll_ref,6)
    
    #calculate at command reference adn direction
    print( pitch_ref, roll_ref,u[0])
    drone.send(at.PCMD(at.PCMD.flag.progressive, roll_ref, pitch_ref, 0, 0))
    
    #record results in a text file
    file.write( str(drone.navdata.demo.theta)+" "+str(drone.navdata.demo.phi)+" "+str(u[0])+" "+str(e[0])+"\n") 
    
    #data=""+str(x_angle)+" "+str(y_angle)+" "+str(x_off)+" "+str(y_off)+"\n"
    #file.write(data)
    time.sleep(0.05)
    count=count+1
    
file.close()
drone.land()
sys.exit()

