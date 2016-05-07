#!/usr/bin/env python
"""
This is a wrapper class for the actions that are present 
in the HTN. 
Actions can etither be primitive or non-primitive. You can
combine 2 actions to make them non-primitive

Actions use Items as inputs and outputs

This is a base class that can be extended to create custom 
actions
"""
__author__ =  'Aaron St. Clair <astclair@cc.gatech.edu>'
__version__=  '0.1'
__license__ = 'BSD'

import copy

import roslib; roslib.load_manifest('tablebot_heres_how_action_executor')
# Brings in the SimpleActionClient
import actionlib
#import tablebot_heres_how_action_executor
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from tablebot_heres_how_action_executor.msg import  ExecuteGoal,ExecuteAction
from std_msgs.msg import Empty,String


class Action(object):

    def __init__(self, name, task_type='primitive', inputs=[],outputs=[]):
        # A human-reable name for the object
        self.name = name

        #the type of action primitive or non-primitive
        self.type=task_type

        # The inputs to the system. (These are part of the Slot class. spec'd below)
        self.inputs = inputs

        # The outputs to the system (These are also part of the Slot class)
        self.outputs = outputs

        #if this is a non-primitive task it could have
        self.subtasks=[]

        #there are grouped subtasks which behave differently during 
        #execution as they might share input
        self.groupedSubtasks=False


    #we have to copy over the inputs and outputs of each of the subtasks
    def addSubtask(self,subtask):
        #copy the inputs but leave out the specifics of the name       
        inputs=[]
        outputs=[]
        for input in subtask.inputs:
            inputs.append(copy.copy(input))
            #inputs[-1].slot_name=None
    	self.inputs.extend(inputs)
        for output in subtask.inputs:
            outputs.append(copy.copy(output))
            #outputs[-1].slot_name=None
    	self.outputs.extend(outputs)
    	self.subtasks.append(subtask)

    '''
    helper method: gets a list of input names for this particular action
    @return a list of slot names
    '''
    def getSlotNames(self):
        output= [] 
        for input in self.inputs:
            if input.slot_name: 
                output.append(input.slot_name)
        return output


    '''
    This method groups the current action with another action
    @return The grouped action
    '''
    def groupWith(self,action):
    	groupedAction=Action (self.name+" & "+action.name,task_type='learned')
        groupedAction.addSubtask(self)
        groupedAction.addSubtask(action)
        used= [0] * len(action.inputs)
        #depending on the level go through each 1 and see if they match
        for output in self.outputs:
            for i,input in enumerate(action.inputs):
                if action.inputs[i].type==output.type and not used[i]:
                    used[i]=1
        #go through all the outputs used in the second task and remove them
        #grouped actions will never have outputs from first based on groupability definition
        groupedAction.outputs=[groupedAction.outputs[i] for i,x in enumerate(self.outputs) if not x.name==groupedAction.outputs[i].name]
        new_inputs=[]
        for i,use in enumerate(used):
            #slot is in use internally, no need to save
            if not use:
                new_inputs.append(groupedAction.inputs[len(self.inputs)+i])

        groupedAction.inputs=groupedAction.inputs[:len(self.inputs)]
        groupedAction.inputs.extend(new_inputs)
        groupedAction.groupedSubtasks=True
        return groupedAction


    '''
    Given a series of input names we match them to the correct 
    input in the system. 
    If the matching works we can execute the task
    '''
    def matchSlots(self,inputs):
        for input,i in enumerate(inputs):
            print (input.compare(self.inputs[i]))

    '''
    This method is overriden in subclasses executing with inputs.
    At this point of execution the slots should be filled with all information
    needed to execute.
    We take an object of the world in case we have to manipulate it
    It returns the required outputs for the function & checks if the output
    seems to be correct.
    In this method we are doing the non-primitive execution as it groups to 
    primitive
    @return success,info required
    '''
    def execute(self,inputs,world):
        #execute the subtasks with part of the input 
        #unless grouped then all subtasks share input
        current_input_point=0
        outputs=[]
        for subtask in self.subtasks:
            if self.groupedSubtasks:
                for i,input in enumerate(self.inputs):
                    if(i<len(subtask.inputs)):
                        subtask.inputs[i].slot_name=self.inputs[i].name
                success,reason=subtask.execute(inputs,world)
            else:
                sub_inputs=inputs[current_input_point:current_input_point+len(subtask.inputs)]
                for i,input in enumerate(sub_inputs):
                    if(i<len(subtask.inputs)):
                        subtask.inputs[i].slot_name=sub_inputs[i].name
                success,reason=subtask.execute(sub_inputs,world)

            current_input_point+=len(subtask.inputs)

            #if any one fails then the whole thing fails
            #TODO probably need to run an undo or something on the physical side
            if not success:
                return success,reason
            elif reason:
                outputs.append(reason)
        #if its just one make that the output
        if(len(outputs)<2 and len(outputs)>0):
            outputs=outputs[0]
    	return True,reason



#pick up an item into the robots hands. It outputs the item that it has picked up
class Pickup(Action):
    def __init__(self):
        pickup_object=Slot('pickup','Item') 
        super(Pickup,self).__init__('Pick up','primitive',[pickup_object],[pickup_object])

    def execute(self,inputs,world):
        #base failure cases
        if not world.holding == None:
            return False,"You cannot pick up when Tablebot is holding an object"
        if inputs[0].manipulable==False:
            return False,inputs[0].name+" item is not manipulable. (You cannot pick up items in the lunchbox)"
        
        # Waits until the action server has started up and started
        # listening for goals.
         # client.wait_for_server() 
        # Creates a goal to send to the action server.
        goal = [String(input.name) for input in inputs]
        message=ExecuteGoal(action =String(self.name),inputs=goal)
         # Sends the goal to the action server.
        world.client.send_goal(message)
     
        # Waits for the server to finish performing the action.
        world.client.wait_for_result()
        # Prints out the result of executing the action
        if world.client.get_result().success:
            self.inputs[0].slot_name=inputs[0].name
            self.outputs[0].slot_name=inputs[0].name
            world.holding=inputs[0]
            print world.holding
        else :
            return False,"Sorry. We think that we failed to pick the object up. Please try again"
        # @TODO ROS things to make the actual pick up get called
        return True,inputs[0]

#pick up an item into the robots hands. It outputs the item that it has picked up
class Store(Action):
    def __init__(self):
        #store an object is only possible if we are holding the object in question
        store_object=Slot('store','Item',lambda world,input: (True if world.holding.name==input  else False) if world.holding else False) 
        store_container=Slot('store','Container') 
        
        super(Store,self).__init__('Store','primitive',[store_object,store_container])

    def execute(self,inputs,world):
        if world.holding == None:
            return False,"The robot is not holding anything"
        if not inputs or len(inputs) == 0:
            return False,"There is not input"
        if not world.holding.name == inputs[0].name:
            return False,"You cannot store when Tablebot is not holding that object"

        goal = [String(input.name) for input in inputs]
        message=ExecuteGoal(action =String(self.name),inputs=goal)
         # Sends the goal to the action server.
        world.client.send_goal(message)
     
        # Waits for the server to finish performing the action.
        world.client.wait_for_result()
     
        # Prints out the result of executing the action
        if world.client.get_result().success:
            world.holding=None
            inputs[1].addItem(inputs[0])
            inputs[0].manipulable=False
            inputs[0].inside=inputs[1]
            self.inputs[0].slot_name=inputs[0].name
            self.inputs[1].slot_name=inputs[1].name
            return True,None
        else:
            return False,"Sorry. That may not have got stored properly. Please try again"

        # @TODO ROS things to make the actual pick up get called

        



'''
This is the equivalent of the Slot class in DISCO
It describes the slots in a given action which are filled.
'''
class Slot(object):
    def __init__(self,name,slot_type,criterion=None,slot_name=None):
        self.name=name #examples of input name can be pickup object
        self.type=slot_type #examples of input types can be an item or a container
        self.slot_name=slot_name #if the name is None then it is considered unspecified. When name is specified then it can be compared
        self.criterion=criterion
    #compare this input object with another one to see if they are the same
    #the level gives the level to which you want these 2 slots to be compared. They can be compared with Name, Type and Slot Name
    def compare(self,inputs,level='Name'):
        level=level.lower().trim()
        if(level == 'name'):
            if self.name==inputs.name:
                return True
        elif(level == 'type'):
            if self.name==inputs.name and self.type==inputs.type:
                return True
        elif(level == 'slot name'):
            if self.name==inputs.name and self.type==inputs.type and self.slot_name == inputs.slot_name:
                return True
        return False

