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
from heres_how_msgs.msg import WebInterfaceButton,WebInterfaceInput,WebInterfaceExecuteAction,WebInterfaceQuestion,WebInterfaceQuestionResponse
from rail_manipulation_msgs.msg import SegmentedObjectList
from std_msgs.msg import Empty,String
from std_srvs.srv import Empty as EmptySrv
from rospkg import RosPack
import copy
import roslib; roslib.load_manifest('tablebot_heres_how_action_executor')
# Brings in the SimpleActionClient
import actionlib
#import tablebot_heres_how_action_executor
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from tablebot_heres_how_action_executor.msg import  ExecuteGoal,ExecuteAction

import json
import time
import HTMLParser
from utils.HTN import HTN

rospack = RosPack()

'''
---------------------
      CONSTANTS
---------------------
'''
SAVE_FOLDER='save'
ERROR_LOG_FOLDER='errorlog'

COMMAND_FILE=rospack.get_path("pydisco")+'/utils/commands.json'

ITEMS_FILE=rospack.get_path("pydisco")+'/utils/items.json' #file with a list of items and containers in 
LOGGING = True

class Object(object):
    pass

'''
    This will listen to topics from ROS and execute appropriate tasks.
    It will listen for buttons being pressed on the web so that it can pass information
'''
class WebInterface(object):
    def __init__(self,items):

        self.client = actionlib.SimpleActionClient('/web_interface/execute_primitve_action', ExecuteAction)
        self.client.wait_for_server()

        self.htn=HTN(items,self.client);

        #this is the topic used to send the current state of the HTN to the user
        self.htnDisplayTopic=rospy.Publisher("web_interface/htn", String,queue_size=10)

        self.questionTopic=rospy.Publisher("web_interface/question", WebInterfaceQuestion,queue_size=1)
        #Are we asking a question
        #Currently only Grouping supported but we may have to add teach new task
        self.currentQuestion=None

        #this is written to file at the end
        self.log= {}

        #stores a copy of the HTN at every timestep
        #used for the UNDO feature
        self.htnAtTimeStep=[]

    #this is run when a particular button is clicked on the Web Interface
    def button_clicked(self,message):
        if(message.button=='teachNewTask'):
            if LOGGING:
                print("Executing teach new task")
            #if adding is successful
            if not self.htn.addNewTask(message.parameters[0]):
                self.ask_question({'question':'That Task Name is already in use. Please pick a different one','answers':[]})
            else:
                if LOGGING:
                    print self.htn.display()

                self.write_log('task start',{
                    'taskName':message.parameters[0]
                })     
                self.htnDisplayTopic.publish(self.htn.display())
        #this refers to completing of a subtask.
        elif(message.button=='taskComplete'):
            self.write_log('task complete',{
                'taskName':self.htn.tree[self.htn.currentSubtask].name
            })     
            self.htn.saveCurrentSubtask()
        #undo's current task
        elif(message.button=='undo'):
            #check if we are executing something
            self.undo()
        #saves 
        elif(message.button=='finishTask'):
            self.write_log('end',{
                'taskName':self.htn.tree[self.htn.currentSubtask].name
            })     
            self.save()
        #an update calls the display function which sends the HTN as it stands to ROS
        elif(message.button=='updateHTN'):
            if LOGGING:
                print self.htn.display()
            self.htnDisplayTopic.publish(self.htn.display())
        self.htnAtTimeStep.append(copy.deepcopy(self.htn.tree))

    #ROS topic for receiving executed actions with an array of inputs
    def execute_task(self,message):
        taskName = HTMLParser.HTMLParser().unescape(message.action)
        if LOGGING:
            print( "<Execute Callback> " + taskName )
        inputs =message.inputs; 
        success,isGroupable,errorInfo=self.htn.executeTask(taskName, inputs);
        #Ask questions about grouping and about substitution
        if((not success) and errorInfo['reason']=='match fail'):
            #one or more of the inputs might have failed to register
            alternatives=self.htn.world.findAlternatives(errorInfo['failed_input'])
            self.currentQuestion={'name':'Substitution','message':message,'failed_input':errorInfo['failed_input'],'options':alternatives}
            self.ask_question({'question':'Substitution: '+errorInfo['failed_input']+' could not be found. Would you instead like to try','answers':alternatives})
        elif(isGroupable):
            self.currentQuestion={'name':'Grouping','options':['yes','no']}
            self.ask_question({'question':'Do you wish to group the last 2 subtasks into a single task?','answers':['Yes','No']})
        elif(not success):
            #This is not a question as it has no answers but it does point out why the user failed to run the task
            self.ask_question({'question':str(errorInfo['reason']),'answers':[]})

        self.htnDisplayTopic.publish(self.htn.display())
        self.htnAtTimeStep.append(copy.deepcopy(self.htn.tree))

        #add something to the log
        self.write_log('execute',{
            'inputs':message.inputs,
            'taskName':taskName,

        })        

        if LOGGING:
            print self.htn.display()
            print("</Execute Callback>")


    #send a question to the ROS topic here
    #format of a message object {'question':'Do you ...','answers':['yes','no','..']}
    def ask_question(self,message):
        wb=WebInterfaceQuestion()
        wb.question=message['question']
        wb.answers=message['answers']
        self.questionTopic.publish(wb)
        

    #get a response from a question
    def get_response(self,message):
        answer=message.answer
        #if we are asking a question. Sometimes we might just send information and user says ok back
        if(self.currentQuestion):
            if LOGGING:
                print self.currentQuestion
                print message
            if self.currentQuestion['name']=='Grouping':
                if(answer.lower()=='yes'):
                    self.htn.groupLastTasks()
            elif self.currentQuestion['name']=='Substitution':
                if not (answer.lower()=="none. undo!" or answer.lower()=='no alternatives detected, okay.'):
                    inputs=self.currentQuestion['message'].inputs
                    new_items = [answer if x==self.currentQuestion['failed_input'] else x for x in inputs]
                    self.currentQuestion['message'].inputs=new_items
                    self.execute_task(self.currentQuestion['message'])

            self.htnDisplayTopic.publish(self.htn.display())
            #add something to the log
            self.write_log('question',{
                'question':self.currentQuestion['name'],
                'options':self.currentQuestion['options'],
                'answer':answer
            })

        #save your response from HTN
        self.htnAtTimeStep.append(self.htn.getHTNState())
        self.currentQuestion=None

    def undo(self):
        if len(self.htnAtTimeStep)>1:
            self.htn.tree=self.htnAtTimeStep[-1];

    #get all the actions in a particular type
    def actions(self,request):
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
        request.action=HTMLParser.HTMLParser().unescape(request.action)
        print request.action
        inputs=self.htn.getInputsForAction(request.action)       
        result={}
        result['inputs']=[]

        for input in inputs:
            result['inputs'].append(WebInterfaceInput())
            result['inputs'][-1].type=input['type']
            result['inputs'][-1].objects=input['objects']
        return result

    #run when a list of objects is in the world. This then updates the 

    #list in World.py
    def objects_segmented(self,message):
        #get the list of objects
        self.htn.world.refreshItems(message.objects)

    #type can be error,timestep,questions,undo
    #data a generic object fill with what you like
    def write_log(self,type,data,getWorldState=True,getHTNState=True):
        #this is a new never before seen key. Create an array of entries for it
        timestamp=time.time()
        if not type in self.log:
            self.log[type]=[]
        self.log[type].append({'data':data,'timestamp':timestamp,'word_state':[],'htn_state':[]})
        if getWorldState:
            self.log[type][-1]['word_state']=self.htn.getCurrentWorldState()
        if getHTNState:
            self.log[type][-1]['htn_state']=self.htn.getHTNState()

    def save(self):
        print self.log
        with open(SAVE_FOLDER+'/'+str(time.time())+'.json', 'a+') as outfile:
            json.dump(self.log, outfile)
        segmenatation = rospy.ServiceProxy('/rail_segmentation/segment', EmptySrv)
        segmenatation()

        self.htn.reset()


if __name__ == '__main__':   
    rospy.init_node('trains_htn_planner', anonymous=False)
    with open(ITEMS_FILE) as item_file:    
        items= json.load(item_file)
        web=WebInterface(items)
        rospy.Subscriber("web_interface/button", WebInterfaceButton,web.button_clicked)
        rospy.Service('web_interface/action_inputs', WebInterfaceActionInputs, web.action_inputs)
        rospy.Service('web_interface/actions', WebInterfaceActions, web.actions)
        rospy.Subscriber('web_interface/execute_action', WebInterfaceExecuteAction, web.execute_task)
        rospy.Subscriber('web_interface/question_response', WebInterfaceQuestionResponse, web.get_response)
        rospy.Subscriber('object_recognition_listener/recognized_objects', SegmentedObjectList, web.objects_segmented)
    #Call segmenatation
    segmenatation = rospy.ServiceProxy('/rail_segmentation/segment', EmptySrv)
    segmenatation()

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
