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
---------------------
      CONSTANTS
---------------------
'''
SAVE_FOLDER='save'
ERROR_LOG_FOLDER='errorlog'

COMMAND_FILE='commands.json'
ITEMS_FILE='items.json' #file with a list of items and containers in 


'''
    This will listen to topics from ROS and execute appropriate tasks.
    It will listen for buttons being pressed on the web so that it can pass information
'''
class WebInterface(object):
    def __init__(self,items):
        #     Topic btnTopic = new Topic(ros, "web_interface/button", "heres_how_msgs/WebInterfaceButton"); call button clicked
        #Topic executeTopic = new Topic(ros, "web_interface/execute_action", "heres_how_msgs/WebInterfaceExecuteAction"); call execute task
        #Topic segmentedObjectsTopic = new Topic(ros, "object_recognition_listener/recognized_objects", "rail_manipulation_msgs/SegmentedObjectList") objects_segmented

        self.htn=HTN(items);

        #Are we asking a question
        #Currently only Grouping supported but we may have to add teach new task
        self.currentQuestion=None

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
        print( "<Execute Callback> " + taskName )
        inputs =message["inputs"];
        success,isGroupable,errorInfo=self.htn.executeTask(taskName, inputs);

        #Ask questions about grouping and about substitution
        if((not success) and errorInfo['reason']=='match fail'):
            #one or more of the inputs might have failed to register
            self.currentQuestion={'name':'Substitution','message':message,'failed_input':errorInfo['failed_input']}
            self.ask_question({'question':errorInfo['failed_input']+' could not be found. Would you instead like to try','answers':self.htn.world.findAlternatives(errorInfo['failed_input'])})
        elif(isGroupable):
            self.currentQuestion={'name':'Grouping'}
            self.ask_question({'question':'Do you wish to group the last 2 subtasks into a single task?','answers':['yes','no']})
        elif(not success):
            #This is not a question as it has no answers but it does point out why the user failed to run the task
            self.ask_question({'question':errorInfo,'answers':[]})

        self.htn.display()
        print("</Execute Callback>")

    #send to the ROS topic here
    #TODO make this post to a ROS Topic
    def ask_question(self,message):
        print message
        pass

    def get_response(self,message):
        answer=message['answer']
        if self.currentQuestion['name']=='Grouping':
            if(answer=='yes'):
                self.htn.groupLastTasks()
        elif self.currentQuestion['name']=='Substitution':
            inputs=self.currentQuestion['message']['inputs']
            new_items = [answer if x==self.currentQuestion['failed_input'] else x for x in inputs]
            self.currentQuestion['message']['inputs']=new_items
            self.execute_task(self.currentQuestion['message'])
        self.currentQuestion=None

    def objects_segmented(self,message):
        self.htn.world.refreshItems(message['objects'])

with open(ITEMS_FILE) as item_file:    
    items= json.load(item_file)
    web=WebInterface(items)

if __name__ == '__main__':
    rospy.Subscriber("web_interface/button", "heres_how_msgs/WebInterfaceButton",web.execute_task);
    rospy.spin()
#we're not using ROS pick up the commands from a file
else:
    with open(COMMAND_FILE) as command_file:    
        data = json.load(command_file)
        #executing a series of commands from a JSON file instead of doing this through ROS for now
        for datum in data:
            if(datum['type']=='button_clicked'):
                web.button_clicked(datum['message'])
            elif(datum['type']=='execute_task'):
                web.execute_task(datum['message'])
            elif(datum['type']=='recognized_objects'):
                web.objects_segmented(datum['message'])
            elif(datum['type']=='question_response'):
                web.get_response(datum['message'])
# if __name__ == '__main__':
#     rospy.init_node('heres_how', anonymous=False)
#     w = World()
#     rospy.on_shutdown(shutdown)
#     rospy.spin()
