#!/usr/bin/env python
"""
This is a wrapper class for the HTN. This is an HTN create.
It can save the HTN and store it and add and execute tasks 

NOTE: When executing a task a Slot gets put in the new copy
of the Task with the object name. Make sure when grouping 
to remove this.

"""
__author__ =  'Aaron St. Clair <astclair@cc.gatech.edu>'
__version__=  '0.1'
__license__ = 'BSD'

import copy

from Action import Action,Pickup,Store,Slot
from Container import Container
from Item import Item
from World import World
class HTN(object):

    def __init__(self,items):
        self.tree=[] #the tree is an array of actions starting from the top and going to the bottom
        self.actionsPerformed=[] #maintains the list of Actions executed so we can undo
        self.actions={} #the set of actions that the user can use 
        self.currentSubtask=-1 #the ID of the current task that is being executed

        #add out primary tasks into the list of actions
        pickup = Pickup()
        store  = Store()
        self.actions={"Pick up":pickup,"Store":store}

        self.world = World(items) #stores the information about the world

    #This will add a new task into the current state of the HTN
    def addNewTask(self,taskName):
        #this creates a new topic with no inputs or outputs. As subtasks are put under it the number of inputs and outputs should increase
        newTask=Action(taskName) #add this task with no inputs and outputs
        self.tree.append(newTask)
        self.currentSubtask=len(self.tree)-1 #change current task to this new task

    #find the correct action and then check if the inputs match
    #if they do, this runs the action and adds it to the current point 
    #in the tree
    #@return if execution was successful
    def executeTask(self,taskName,inputs):
        #check if this is in the set of actions the user can use.
        #if it is add it as a subtask of the current highlighted task
        if self.actions[taskName]:
            #make a copy so that you can fill it in with the correct inputs
            action= copy.deepcopy(self.actions[taskName])
            #array of Item and containers that go in a slot
            final_input=[]
            #convert the inputs from a series of strings to a series of Slot classes
            #for each input find the appropriate slot and put it into it
            for input in inputs:
                #if this does not work we cannot figure out where to put this input, so exec fails
                if not self.world.makeSlot(input,action.inputs):
                    return False
                #there is a slot add to input
                else:
                    final_input.append(self.world.getObject(Input))
            
            #add the task to the current task that is highlighted-
            self.tree[self.currentSubtask].addSubtask(action)
            #execute the task
            action.execute(final_input,world)
            return True
        else :
            print 'The  %s action does not exist'% (taskName)
            return False

    #ending a subtask. Over here we add this task to the complex actions
    def saveCurrentSubtask(self):
        #self.actions.append()
        action= copy.deepcopy(self.tree[self.currentSubtask])
        self.actions.append(action)
        pass

    #this saves the current tree to file and then wipes it 
    #also needs to reset the action queue to only primitive actions
    def save(self):
        pass

    '''
        recursive method that builds the HTN for printing 
        called to get all the subtask
        return style:
           name: 'Task Name [Inputs]'
           focus: true
           defined: false
           decompositions:
            steps:[]
    '''
    def build_htn(self,action):
        output='{'
        output+= "\"name\": \""+(action.name+" "+ ", ".join(action.getSlotNames()))+"\""  #printing name and slots
        output+= ", \"focus\": \""+str(self.tree[self.currentSubtask].name==action.name)+"\"" 
        output+= ", \"defined\": \"true\""  #not sure what defined means so I return true here
        output+= ", \"decompositions\": ["
        for subtask in action.subtasks:
            output+= "{ \"steps\": [" 
            output+=self.build_htn(subtask)
            output+=']}'
        output+=']'
        output+='}'
        return output
 

    '''
        Called when we need to display the HTN
    '''
    def display(self):
        output="{\"data:\":["
        #build an outermost task
        for i,action in enumerate(self.tree):
            if not i==0:
                output+="],[" #reopen the brackets
            output+=self.build_htn(action)

        output+="]}"
        print(output) #TODO this shouldn't be printing but sending to ROS


