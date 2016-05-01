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
from heres_how_msgs.srv import WebInterfaceActionInputs,WebInterfaceActions
from heres_how_msgs.msg import WebInterfaceButton,WebInterfaceInput,WebInterfaceExecuteAction

from std_msgs.msg import Empty,String

import json

from utils.HTN import HTN


'''
---------------------
      CONSTANTS
---------------------
'''
SAVE_FOLDER='save'
ERROR_LOG_FOLDER='errorlog'

COMMAND_FILE='utils/commands.json'
ITEMS_FILE='utils/items.json' #file with a list of items and containers in 
LOGGING = True

class Object(object):
    pass

'''
    This will listen to topics from ROS and execute appropriate tasks.
    It will listen for buttons being pressed on the web so that it can pass information
'''
class WebInterface(object):
    def __init__(self,items):

        self.htn=HTN(items);

        #this is the topic used to send the current state of the HTN to the user
        self.htnDisplayTopic=rospy.Publisher("web_interface/htn", String,queue_size=10)

        self.questionTopic=rospy.Publisher("web_interface/question", String,queue_size=10)
        #Are we asking a question
        #Currently only Grouping supported but we may have to add teach new task
        self.currentQuestion=None

    #this is run when a particular button is clicked on the Web Interface
    def button_clicked(self,message):
        if(message.button=='teachNewTask'):
            if LOGGING:
                print("Executing teach new task")
            self.htn.addNewTask(message.parameters[0])
            if LOGGING:
                print self.htn.display()
            self.htnDisplayTopic.publish(self.htn.display())
        #this refers to completing of a subtask.
        elif(message.button=='TaskComplete'):
            self.htn.saveCurrentSubtask()
        #undo's current task
        elif(message.button=='undo'):
            pass
        #saves 
        elif(message.button=='finishTask'):
            self.htn.save()
        #an update calls the display function which sends the HTN as it stands to ROS
        elif(message.button=='updateHTN'):
            if LOGGING:
                print self.htn.display()
            self.htnDisplayTopic.publish(self.htn.display())

    #ROS topic for receiving executed actions with an array of inputs
    def execute_task(self,message):
        taskName = message.action
        if LOGGING:
            print( "<Execute Callback> " + taskName )
        inputs =message.inputs;
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

        self.htnDisplayTopic.publish(self.htn.display())

        if LOGGING:
            print("</Execute Callback>")

    #send to the ROS topic here
    #TODO make this post to a ROS Topic
    def ask_question(self,message):
        print message
        pass

    #get a response from a question
    def get_response(self,message):
        answer=message.answer
        if self.currentQuestion['name']=='Grouping':
            if(answer=='yes'):
                self.htn.groupLastTasks()
        elif self.currentQuestion['name']=='Substitution':
            inputs=self.currentQuestion['message']['inputs']
            new_items = [answer if x==self.currentQuestion['failed_input'] else x for x in inputs]
            self.currentQuestion['message']['inputs']=new_items
            self.execute_task(self.currentQuestion['message'])
        self.currentQuestion=None

    #get all the actions in a particular type
    def actions(self,request):
        print request.Request
        actions=[]
        if request.Request=='primitive':
            actions=self.htn.getActions('primitive')
        elif request.Request=='learned':
            actions=self.htn.getActions('learned')
        result={}
        result['Actions']=[]
        for action in actions:
            result['Actions'].append(Object())
            result['Actions'][-1].ActionType=action
            result['Actions'][-1].Inputs=", ".join([x.type for x in actions[action].inputs])
        
        return result

    #get the inputs available for a particular action
    #eg. For pick up there will be one input
    def action_inputs(self,request):
        inputs=self.htn.getInputsForAction(request.action)       
        result={}
        result['inputs']=[]

        for input in inputs:
            result['inputs'].append(WebInterfaceInput())
            result['inputs'][-1].type=input['type']
            result['inputs'][-1].objects=input['objects']
        return result

    def objects_segmented(self,message):
        self.htn.world.refreshItems(message['objects'])

with open(ITEMS_FILE) as item_file:    
    items= json.load(item_file)
    web=WebInterface(items)

if __name__ == '__main__':
    
    rospy.init_node('trains_htn_planner', anonymous=False)
    rospy.Subscriber("/web_interface/button", WebInterfaceButton,web.button_clicked)
    rospy.Service('web_interface/action_inputs', WebInterfaceActionInputs, web.action_inputs)
    rospy.Service('web_interface/actions', WebInterfaceActions, web.actions)
    rospy.Subscriber('/web_interface/execute_action', WebInterfaceExecuteAction, web.execute_task)
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
