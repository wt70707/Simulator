#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import sys, select, termios, tty

class SpiriKeyboardTeleopNode:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        state_topic = rospy.get_param("state_topic", "/ground_truth/state")
        state_sub = rospy.Subscriber(state_topic, Odometry, self.state_callback, queue_size=1)
        
        self.hover_height = rospy.get_param("hover_height", 2)
        self.hover_thresh = rospy.get_param("hover_thresh", 0.1)
        
        rospy.init_node('teleop_twist_keyboard')
        self.state = Odometry()
        
        r = rospy.Rate(300)
        key = ''
        
        while not rospy.is_shutdown():
            cmd_vel = Twist()
            if key == '':
                key = self.get_key()
            #print "command: " + str(key)
            err = None
            if key in self.moveBindings.keys():
                if key == 'h':
                    err = (self.hover_height - self.state.pose.pose.position.z)
                    cmd_vel.linear.z = (self.hover_height - self.state.pose.pose.position.z)
                
                elif key=='n':
                    err = -self.state.pose.pose.position.z
                    cmd_vel.linear.z = err
                    
                else:
                    cmd_vel.linear.x = self.moveBindings[key][0]
                    cmd_vel.angular.z = self.moveBindings[key][1]
                    cmd_vel.linear.z = self.moveBindings[key][2]
                    
            elif key in self.speedBindings.keys():
				self.speed = self.speed * self.speedBindings[key][0]
				self.turn = self.turn * self.speedBindings[key][1]

				print "currently:\tspeed %s\tturn %s " % (self.speed, self.turn)

            else:
                if (key == '\x03'):
                    break
                print self.msg
            
            if err == None or (abs(err) < self.hover_thresh):
                key = ''
            
            cmd_vel.linear.x *= self.speed
            cmd_vel.linear.y *= self.speed
            cmd_vel.linear.z *= self.speed
            
            cmd_vel.angular.z *= self.turn
            
            cmd_vel_pub.publish(cmd_vel)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            r.sleep()

    def state_callback(self, msg):
        #print msg
        self.state = msg

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        [rr, rw, rx] = select.select([sys.stdin], [], [], 0.25)
        if sys.stdin in rr: 
            key = sys.stdin.read(1)
        else:
             key = ""
             
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)             
        return key

        
    msg = """
    Reading from the keyboard  and Publishing to Twist!
    ---------------------------
    Moving around:
       u    i    o
       j    k    l
       m    ,    .

    q/z : increase/decrease max speeds by 10%
    w/x : increase/decrease only linear speed by 10%
    e/c : increase/decrease only angular speed by 10%
    p   : move up in z direction
    ;   : move down in z directions
    h   : Hover
    n   : Land
    anything else : stop

    CTRL-C to quit
    """

    moveBindings = {
            '':(0,0,0),
            'i':(1,0,0),
            'o':(1,-1,0),
            'j':(0,1,0),
            'l':(0,-1,0),
            'u':(1,1,0),
            ',':(-1,0,0),
            '.':(-1,1,0),
            'm':(-1,-1,0),
            'p':(0,0,1),
            ';':(0,0,-1),
            'h':(0,0,1),
            'n':(0,0,0),
    }

    speedBindings={
            'q':(1.1,1.1),
            'z':(.9,.9),
            'w':(1.1,1),
            'x':(.9,1),
            'e':(1,1.1),
            'c':(1,.9),
    }
    
    speed = .5
    turn = 1        

if __name__=="__main__":
    spiri_keyboard_teleop_node = SpiriKeyboardTeleopNode()

