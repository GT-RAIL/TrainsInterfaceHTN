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

    #we have to copy over the inputs and outputs of each of the subtasks
    def addSubtask(self,subtask):
        #copy the inputs but leave out the specifics of the name
        
        inputs=[]
        outputs=[]
        for input in self.inputs:
            inputs.append(copy.copy(input))
            inputs[-1].slot_name=None
    	self.inputs.extend(inputs)
        for output in self.inputs:
            inputs.append(copy.copy(output))
            outputs[-1].slot_name=None
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
            else:
                output.append('')
        return output


    '''
    This method groups the current action with another action
    @return The grouped action
    '''
    def groupWith(self,action):
    	pass


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
    '''
    def execute(self,inputs,world):
        #execute the subtasks with part of the input
        current_input_point=0
        for subtask in self.subtask:
            subtask.execute(inputs[current_input_point:current_input_point+len(subtask.inputs)],world)
            current_input_point+=len(subtask.inputs)
    	



#pick up an item into the robots hands. It outputs the item that it has picked up
class Pickup(Action):
    def __init__(self):
        pickup_object=Slot('pickup','Item') 
        super(Pickup,self).__init__('Pick up','primitive',[pickup_object],[pickup_object])

    def execute(self,inputs,world):
        inputs[0].manipulable=False
        world.holding=inputs[0]
        # @TODO ROS things to make the actual pick up get called
        return inputs[0]

#pick up an item into the robots hands. It outputs the item that it has picked up
class Store(Action):
    def __init__(self):
        store_object=Slot('store','Item') 
        store_container=Slot('store','Container') 
        super(Store,self).__init__('Store','primitive',[store_object,store_container])

    def execute(self,inputs,world):
        world.holding=None
        inputs[1].addItem(inputs[0])
        # @TODO ROS things to make the actual pick up get called

        pass



'''
This is the equivalent of the Slot class in DISCO
It describes the slots in a given action which are filled.
'''
class Slot(object):
    def __init__(self,name,slot_type,slot_name=None):
        self.name=name #examples of input name can be pickup object
        self.type=slot_type #examples of input types can be an item or a container
        self.slot_name=slot_name #if the name is None then it is considered unspecified. When name is specified then it can be compared

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

