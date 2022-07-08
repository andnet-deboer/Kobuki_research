#!/usr/bin/env python
from __future__ import print_function
from __future__ import division
from time import sleep
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import Led
from std_msgs.msg import Float32
from math import radians
import time
import rospy
from kobuki_msgs.msg import BumperEvent


#MOVE 
def move( id,linear, angular):
    """
    Function to move robot

    Parameters:(id, linear, angular)

    -id (--integer): identifier which determines which robot will move |0 -> Arlo|1 -> Tobias|2 -> Estevan|

    -linear (--float): linear vector of the robot controls the speed and direction 
                     postive is forward and negative is backwards

    -angular (--float): angular vector of the robot controls the speed and direction 
                     postive is clockwise and negative is counter-clockwise
                    
    """
    arlocmd_vel = rospy.Publisher('/arlo/mobile_base/commands/velocity', Twist, queue_size=10)
    tobiascmd_vel = rospy.Publisher('/tobias/mobile_base/commands/velocity', Twist, queue_size= 10)
    estevancmd_vel = rospy.Publisher("/estevan/mobile_base/commands/velocity", Twist,queue_size=10)

    #move
    move_cmd = Twist()
    move_cmd.linear.x = linear
    move_cmd.angular.z = radians(angular)

    while not rospy.is_shutdown():
        if id == 0:
            arlocmd_vel.publish(move_cmd)
        if id == 1 :
            tobiascmd_vel.publish(move_cmd)
        if id == 2:
            estevancmd_vel.publish(move_cmd)
        break
#MOVE_BACK
def moveback(int):
    """
    Function to move robot backwards

    -int (--integer): identifier which determines which robot will move |0 -> Arlo|1 -> Tobias|2 -> Estevan|

                    
    """
    arlocmd_vel = rospy.Publisher("/arlo/mobile_base/commands/velocity", Twist, queue_size=19)
    tobiascmd_vel = rospy.Publisher("/tobias/mobile_base/commands/velocity", Twist,queue_size=10)
    estevancmd_vel = rospy.Publisher("/estevan/mobile_base/commands/velocity", Twist,queue_size=10)
    
    #move back
    moveback_cmd = Twist()
    moveback_cmd.linear.x = -0.2

    while not rospy.is_shutdown():
        if int== 0:
            arlocmd_vel.publish(moveback_cmd)
            sleep(1)
        if int == 1 :
            tobiascmd_vel.publish(moveback_cmd)
            sleep(1)
        if int == 2:
            estevancmd_vel.publish(moveback_cmd)
            sleep(1)
           
        break

class avoid:
    #INTIALIZES SELF VARIABLES
    def __init__(self):

         #------------------------------------ Estevan
        self.estevanleftsensor = 0
        self.estevanrightsensor = 0
        self.estevanbumper = False

        #------------------------------------- Arlo
        self.arloleftsensor = 0
        self.arlorightsensor = 0
        self.arlobumper = False


        #------------------------------------ Tobias
        self.tobiasleftsensor = 0
        self.tobiasrightsensor = 0
        self.tobiasbumper = False

        #------------------------------------ System Variables
        self.distance = 18
        self.turnangle = 280
        self.velocity = 0.25

#======================== ESTEVAN ===========================#

    #RETRIEVES ESTEVAN LEFT ULTRASONIC SENSOR DATA
    def estevanleftsensor_callback(self, msg):
        # "Store" message received
        self.estevanleftsensor = msg.data
        # Compute stuff.
        self.estevancompute()
   
    #RETRIEVES ESTEVAN RIGHT ULTRASONIC SENSOR DATA
    def estevanrightsensor_callback(self, msg):
        # "Store" the message received.
        self.estevanrightsensor = msg.data
        # Compute stuff.
        self.estevancompute()
       
    #RETRIEVES ESTEVAN BUMPER DATA
    def estevanbumper_callback(self, msg):
       self.estevanbumper = msg.state
       self.estevancompute()

    def estevancompute(self):

        self.mainloop(2, self.estevanleftsensor, self.estevanrightsensor, self.estevanbumper)
   

#======================= Tobias ===========================# 
    #RETRIEVES TOBIAS LEFT ULTRASONIC SENSOR DATA
    def tobiasleftsensor_callback(self, msg):
        # "Store" message received
        self.tobiasleftsensor = msg.data
        # Compute stuff.
        self.tobiascompute()

    #RETRIEVES TOBIAS RIGHT ULTRASONIC SENSOR DATA
    def tobiasrightsensor_callback(self, msg):
        # "Store" the message received.
        self.tobiasrightsensor = msg.data
        # Compute stuff.
        self.tobiascompute()
  

    #RETRIEVS TOBIAS BUMPER DATA
    def tobiasbumper_callback(self, msg):
       self.tobiasbumper = msg.state
       self.tobiascompute()

    def tobiascompute(self):
        
        
        self.mainloop(1, self.tobiasleftsensor, self.tobiasrightsensor, self.tobiasbumper)

#======================= Arlo ===========================# 
    #RETRIEVES Arlo LEFT ULTRASONIC SENSOR DATA
    def arloleftsensor_callback(self, msg):
        # "Store" message received
        self.arloleftsensor = msg.data
        # Compute stuff.
        self.arlocompute()

    #RETRIEVES Arlo RIGHT ULTRASONIC SENSOR DATA
    def arlorightsensor_callback(self, msg):
        # "Store" the message received.
        self.arlorightsensor = msg.data
        # Compute stuff.
        self.arlocompute()
  
    #RETRIEVS ARLO BUMPER DATA
    def arlobumper_callback(self, msg):
       self.arlobumper = msg.state
       self.arlocompute()

    def arlocompute(self):
        
        self.mainloop(0, self.arloleftsensor, self.arlorightsensor, self.arlobumper)



    def mainloop(self,id, leftsensor, rightsensor, bumper):

       if leftsensor is not None and rightsensor is not None: 

        if (bumper == BumperEvent.PRESSED):
            bumperpressed = True
                
            while bumperpressed == True:     
                moveback(id)
                move(id,0,150)
                sleep(1)
                break
            bumperpressed = False 
       
        elif leftsensor < self.distance:
            move(id,0,-self.turnangle)
            print(leftsensor)

        elif rightsensor < self.distance:
            move(id,0,self.turnangle)
            print(rightsensor)

        else:
            move(id,self.velocity,0)
            print("left:", leftsensor,"  ", "right: " ,rightsensor)


if __name__ == '__main__':
    rospy.init_node('avoid')

    avoid = avoid()

    #ARLO SUBSCRIBERS
    rospy.Subscriber('/arlo/ultrasonicsensor_1', Float32 , avoid.arloleftsensor_callback)
    rospy.Subscriber('/arlo/ultrasonicsensor_2', Float32, avoid.arlorightsensor_callback)
    rospy.Subscriber("/arlo/mobile_base/events/bumper", BumperEvent, avoid.arlobumper_callback)
    
    #TOBIAS SUBSCRIBERS
    rospy.Subscriber('/tobias/ultrasonicsensor_1', Float32 , avoid.tobiasleftsensor_callback)
    rospy.Subscriber('/tobias/ultrasonicsensor_2', Float32, avoid.tobiasrightsensor_callback)
    rospy.Subscriber("/tobias/mobile_base/events/bumper", BumperEvent, avoid.tobiasbumper_callback)
    
    #ESTEVAN SUBSCRIBERS
    rospy.Subscriber('/estevan/ultrasonicsensor_1', Float32 , avoid.estevanleftsensor_callback)
    rospy.Subscriber('/estevan/ultrasonicsensor_2', Float32, avoid.estevanrightsensor_callback)
    rospy.Subscriber("/estevan/mobile_base/events/bumper", BumperEvent, avoid.estevanbumper_callback)
    
    rospy.spin()

