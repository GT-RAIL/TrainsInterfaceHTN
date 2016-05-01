#!/usr/bin/env python
"""HTN management node for lunchpacking task

This node receives input from an RMS interface and provides lists
of available actions and inputs. As actions are executed a task 
tree is built and published. 
"""
__author__ =  'Aaron St. Clair <astclair@cc.gatech.edu>'
__version__=  '0.1'
__license__ = 'BSD'

#from __future__ import print_function


#ROS things
import sys
import os
import signal
import subprocess
import rospy
import rospkg
from rail_user_queue_manager.msg import Queue
from std_msgs.msg import Empty

import json

from utils.HTN import HTN
'''
    This will listen to topics from ROS and execute appropriate tasks.
    It will listen for buttons being pressed on the web so that it can pass information
'''
class WebInterface(object):
    def __init__(self):
        #     Topic btnTopic = new Topic(ros, "web_interface/button", "heres_how_msgs/WebInterfaceButton"); call button clicked
        #Topic executeTopic = new Topic(ros, "web_interface/execute_action", "heres_how_msgs/WebInterfaceExecuteAction"); call execute task
        self.htn=HTN();

    #this is run when a particular button is clicked on the Web Interface
    def button_clicked(self,message):
        if(message['button']=='teachNewTask'):
            print("Executing teach new task")
            self.htn.addNewTask(message['parameters'][0])
        #this refers to completing of a subtask.
        elif(message['button']=='TaskComplete'):
            self.htn.saveCurrentSubtask()
        #undo's current task
        elif(message['button']=='undo'):
            pass
        #saves 
        elif(message['button']=='finishTask'):
            self.htn.save()
        #an update calls the display function which sends the HTN as it stands to ROS
        elif(message['button']=='updateHTN'):
            self.htn.display()

    #ROS topic for receiving executed actions with an array of inputs
    def execute_task(self,message):
        taskName = message["action"]
        print( "<Execute Callback>" + taskName )
        inputs =message["inputs"];
        self.htn.executeTask(taskName, inputs);
        self.htn.display()
        print("</Execute Callback>")


web=WebInterface()
with open('commands.json') as data_file:    
    data = json.load(data_file)
    #executing a series of commands from a JSON file instead of doing this through ROS for now
    for datum in data:
        if(datum['type']=='button_clicked'):
            web.button_clicked(datum['message'])
        elif(datum['type']=='execute_task'):
            web.execute_task(datum['message'])
# if __name__ == '__main__':
#     rospy.init_node('heres_how', anonymous=False)
#     w = World()
#     rospy.on_shutdown(shutdown)
#     rospy.spin()
