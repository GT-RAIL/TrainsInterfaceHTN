#!/usr/bin/env python
"""Simple node to spawn and re-span the heres-how web_interface HTN node

This node monitors the rail_user_queue_manager/queue and kills/restarts
a new instance of heres_how when the active user changes. This is 
necessary since heres_how is packaged as a jar file. 
"""
__author__ =  'Aaron St. Clair <astclair@cc.gatech.edu>'
__version__=  '0.1'
__license__ = 'BSD'

import sys
import os
import signal
import subprocess
import rospy
import rospkg
from rail_user_queue_manager.msg import Queue
from std_msgs.msg import Empty


class HeresHowSpawner:

    def __init__(self):
        '''Initialize ros subscriber'''
         
        # currentUser = -1 signifies a freshly-started process is waiting
        # an the queue is empty
        self.currentUser = -1 

        self.cmd = "java -jar heres-how-web-interface.jar"
        rospack = rospkg.RosPack()
        self.cwd = rospack.get_path("tablebot_heres_how_action_executor")

        # notification topic for alerts on heres-how restart
        self.pub = rospy.Publisher("web_interface/heres_how_init", Empty, queue_size=10)

        self.startProcess()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/rail_user_queue_manager/queue",
            Queue, self.callback,  queue_size = 1)

    def startProcess(self):
        self.process = subprocess.Popen(self.cmd, cwd=self.cwd, shell=True, preexec_fn=os.setsid)
        rospy.loginfo("Started heres-how-web-interface process")
        self.pub.publish(Empty())

    def stopProcess(self):
        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
        rospy.loginfo("Killed heres-how-web-interface process")

    def restartProcess(self):
        rospy.loginfo("Restarting heres-how process...")
        self.stopProcess();
        self.startProcess();
        rospy.loginfo("Done restarting heres-how process...")

    def callback(self, q):
        '''Callback function of subscribed queue topic. 
        Check to see if active user has changed.'''
        rospy.loginfo("Current user id: %d" % self.currentUser)
        # kill and restart heres-how if the queue is now empty (someone finished)
        if( self.currentUser != -1 and len(q.queue) == 0 ):
            self.restartProcess()
            self.currentUser = -1
        # kill and restart heres-how if a new user has become active
        elif( self.currentUser != -1 and self.currentUser != q.queue[0].user_id ): 
            self.currentUser = q.queue[0].user_id
            self.restartProcess()

        if( self.currentUser == -1 and len(q.queue) > 0):
            self.currentUser = q.queue[0].user_id

def shutdown():
    print "Shutting down heres_how_spawner..."
    h.stopProcess()

if __name__ == '__main__':
    rospy.init_node('heres_how_spawner', anonymous=False)
    h = HeresHowSpawner()
    rospy.on_shutdown(shutdown)
    rospy.spin()
