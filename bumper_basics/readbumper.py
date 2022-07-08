#!/usr/bin/env python
from cmath import pi
from doctest import testfile
from math import radians
from operator import truediv
from pickle import FALSE
from shutil import move
import threading
from tokenize import Double
import rospy
from kobuki_msgs.msg import BumperEvent
from kobuki_msgs.msg import Led
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
import time
from kobuki_msgs.msg import Sound 
from kobuki_msgs.msg import ScanAngle 
from kobuki_msgs.msg import SensorState
from time import sleep


#turn right command
turnright_cmd = Twist()
turnright_cmd.linear.x = 0
turnright_cmd.angular.z = radians(-180)
#turn left command
turnleft_cmd = Twist()
turnleft_cmd.angular.z = radians(180)
#move back command
moveback_cmd = Twist()
moveback_cmd.linear.x = -0.3
#move forward command
moveforward_cmd = Twist()
moveforward_cmd.linear.x = 0.0
#stop command
stop_cmd = Twist()
stop_cmd.linear.x = 0.0

bump = False
global latch 
global turnrightvar
global movebackvar
global turnleftvar 
latch = False
#if bump data is received, process here
#data.bumper: LEFT (0), CENTER (1), RIGHT (2)
#data.state: RELEASED(0), PRESSED(1)
def processBump(data):

    #if (data.state == BumperEvent.PRESSED):
    bump = True
    global turnrightvar
    global movebackvar
    global turnleftvar 
    #turnrightvar =False
    #movebackvar =False
    #turnleftvar =False
    
    bumperdata = rospy.Publisher("estevan/bumperdata", Bool, queue_size=10)

    if (data.state == BumperEvent.PRESSED):

        bumperdata.publish(True)
        
        if data.bumper == 0:
        
            turnonled(1)
            turnrightvar = True

        elif data.bumper == 1:
            
            turnonled(2)
            movebackvar = True

        elif data.bumper == 2:
            
            turnonled(3)
            turnleftvar = True
    else: 
        bump = False 
        turnonled(0)
        bumperdata.publish(False)
    

    if data.bumper == 0:
        
        sound(0)
        if (turnrightvar == True):
            turnright()

    elif data.bumper == 1:
            
        sound(1)
        if (movebackvar == True):
            moveback()

    elif data.bumper == 2:
            
        sound(2)
        if (turnleftvar == True):
            turnleft()

    rospy.loginfo("Bumper Event")
    rospy.loginfo(data.bumper)
    rospy.loginfo(bump)
    bump = False
    #moveforward(bump)

# Method To Move Back and Turn 
def moveback():
    tobiascmd_vel = rospy.Publisher('tobias/mobile_base/commands/velocity', Twist, queue_size=10)
    estevancmd_vel = rospy.Publisher('estevan/mobile_base/commands/velocity', Twist, queue_size=10)
    tobiascmd_vel.publish(moveback_cmd)
    estevancmd_vel.publish(moveback_cmd)
    time.sleep(.9)
    estevancmd_vel.publish(turnleft_cmd) 
    tobiascmd_vel.publish(turnleft_cmd)
    time.sleep(.1)
    

# Method To Turn Left
def turnleft():
    tobiascmd_vel = rospy.Publisher('tobias/mobile_base/commands/velocity', Twist, queue_size=10)
    estevancmd_vel = rospy.Publisher('estevan/mobile_base/commands/velocity', Twist, queue_size=10)
   
    tobiascmd_vel.publish(turnleft_cmd)
    estevancmd_vel.publish(turnleft_cmd)

# Method To Turn Right
def turnright():
    tobiascmd_vel = rospy.Publisher('tobias/mobile_base/commands/velocity', Twist, queue_size=10)
    estevancmd_vel = rospy.Publisher('estevan/mobile_base/commands/velocity', Twist, queue_size=10)
    tobiascmd_vel.publish(turnright_cmd)
    estevancmd_vel.publish(turnright_cmd)
#Method To Stop
def stopcmd():
    tobiascmd_vel = rospy.Publisher('tobias/mobile_base/commands/velocity', Twist, queue_size=10)
    estevancmd_vel = rospy.Publisher('estevan/mobile_base/commands/velocity', Twist, queue_size=10)
   
    tobiascmd_vel.publish(stop_cmd)
    estevancmd_vel.publish(stop_cmd)
#Method To Turn on Leds Parameter int Specifies Color
def turnonled(int):
    tobias_led1 = rospy.Publisher('tobias/mobile_base/commands/led1', Led, queue_size=10)
    estevan_led1 = rospy.Publisher('estevan/mobile_base/commands/led1', Led, queue_size=10)

    if(int == 0):
     estevan_led1.publish(Led.BLACK)
     tobias_led1.publish(Led.BLACK)
    elif(int == 1):
     estevan_led1.publish(Led.GREEN)
     tobias_led1.publish(Led.GREEN)
    elif(int == 2):
     estevan_led1.publish(Led.ORANGE)
     tobias_led1.publish(Led.ORANGE)
    elif(int == 3):
     estevan_led1.publish(Led.RED)
     tobias_led1.publish(Led.RED)
#Method To Have Robot Roam
def roam(data):
    #while not rospy.is_shutdown():
        while(data.state != BumperEvent.PRESSED):
            #moveforward(bump)
            break
#Make Sound
def sound(int):
    estevansound = rospy.Publisher('estevan/mobile_base/commands/sound', Sound, queue_size=10)
    tobiassound = rospy.Publisher('tobias/mobile_base/commands/sound', Sound, queue_size=10)
    estevansound.publish(int)
    tobiassound.publish(int)
#Move Forward 
def moveforward():
    tobiascmd_vel = rospy.Publisher("tobias/mobile_base/commands/velocity", Twist, queue_size=10)
    estevancmd_vel = rospy.Publisher("estevan/mobile_base/commands/velocity", Twist, queue_size=10)
    r = rospy.Rate(3) # 10hz
    while not rospy.is_shutdown():
        tobiascmd_vel.publish(moveforward_cmd)  
        estevancmd_vel.publish(moveforward_cmd)
        rospy.loginfo("moving forward")
        r.sleep()

#def convertdata(data):
    #bumperdata = rospy.Publisher("estevan/bumperdata", Bool, queue_size=1)
    #if (data.state == BumperEvent.PRESSED):
        #bumperdata.publish(True)
    #else:
        #bumperdata.publish(False)
def task():
    print('Starting a task...')
    sleep(1)
    return False
    print('done')
    
def printdata(data):
    #test = "estevan/mobile_base/events/bumper"
    #test = rospy.Subscriber("estevan/bumper", BumperEvent, data)
    #rospy.loginfo(data)
        #continue
    #else:
    global latch
    if(data.data == True):
        latch = True
        #time.sleep(20000)
        #latch = False


    rospy.loginfo(latch)
    if(latch == False):
        moveforward()
    
        
def readbumper():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('readbumper_node', anonymous=False)
    rospy.Subscriber("estevan/mobile_base/events/bumper", BumperEvent, processBump)
    rospy.Subscriber("tobias/mobile_base/events/bumper", BumperEvent, processBump)
    #rospy.Subscriber("estevan/mobile_base/events/bumper", BumperEvent, convertdata)
    rospy.Subscriber("estevan/bumperdata", Bool, printdata)
    #rospy.Subscriber("estevan/mobile_base/events/bumper", BumperEvent, roam)
    #rospy.Subscriber("tobias/mobile_base/events/bumper", BumperEvent, roam)


    
    
    r = rospy.Rate(.5) # 10hz
    while not rospy.is_shutdown():
        #moveforward(False)
        #rospy.Subscriber("estevan/bumperdata", Bool, printdata)
        #printdata()
        #bumperdata = rospy.Publisher("estevan/bumperdata", Bool, queue_size=10)
        #rospy.Subscriber("")
        #global turnrightvar
        #global movebackvar
        #global turnleftvar 
        
        #if (turnrightvar == True):
           # turnright()
           # sleep(2)
           # turnrightvar = False
            
       # if (movebackvar == True):
         #   moveback()
         #   sleep(2)
            #movebackvar = False

        #if (turnleftvar == True):
           # turnleft()
            #sleep(2)
           # turnleftvar = False
      #  global latch
       # if(latch == True):
       #     latch = task()
        #rospy.loginfo(test.callback)
        r.sleep()
    #THIS IS A TEST OF SYNC
    


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    readbumper()

