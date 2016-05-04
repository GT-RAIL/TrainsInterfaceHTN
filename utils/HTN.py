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
        self.items=items
        self.reset()

    def reset(self):
        self.tree=[] #the tree is an array of actions starting from the top and going to the bottom
        self.actionsPerformed=[] #maintains the list of Actions executed so we can undo
        self.actions={} #the set of actions that the user can use 
        self.currentSubtask=-1 #the ID of the current task that is being executed

        #add out primary tasks into the list of actions
        pickup = Pickup()
        store  = Store()
        self.actions={"Pick up":pickup,"Store":store}

        self.world = World(self.items) #stores the information about the world

    def getCurrentWorldState(self):
       return self.world.getCurrentWorldState()

    def getHTNStateRecursive(self,node):
        output={}
        output["name"]=action.name
        output['slots']=(action.getSlotNames())  #printing name and slots
        output['focus']=(self.tree[self.currentSubtask].name==action.name)
        output['decompositions']=[]

        for subtask in action.subtasks:
            output['decompositions'].append(getHTNStateRecursive(subtask))

        return output
 
    def getHTNState(self):
        output=[]
        for i,action in enumerate(self.tree):
            output.append(self.build_htn(action))
        return output

    #This will add a new task into the current state of the HTN
    def addNewTask(self,taskName):
        #this creates a new topic with no inputs or outputs. As subtasks are put under it the number of inputs and outputs should increase
        newTask=Action(taskName) #add this task with no inputs and outputs
        newTask.inputs=[]
        newTask.outputs=[]
        self.tree.append(newTask)
        self.currentSubtask=len(self.tree)-1 #change current task to this new task

    #get all the actions for a particular type
    def getActions(self,type):
        #filter by type
        actions={i: value for i,value in self.actions.iteritems() if value.type==type}
        return actions

    def getInputsForAction(self,action):
        inputs=[]
        for input in self.actions[action].inputs:
            inputs.append({'type':input.type,'objects':self.world.getObjectsByType(input.type,input.criterion)})
        return inputs

    #checks if 2 tasks are groupable
    #the level gives the level to which you want these 2 slots to be compared. They can be compared with  Type and Slot Name
    def isGroupable(self,task1,task2,level="Type"):
        #forgive small typos
        level=level.lower()
        #check that the output sizes match the inputs & that task 1 has an o/p to connect
        if len(task1.outputs)>0:
            if len(task1.outputs)<=len(task2.inputs):
                #depending on the level go through each 1 and see if they match
                for output in task1.outputs:
                    found=False
                    #check if the input is used to fill any other output
                    used= [0] * len(task2.inputs)
                    #go through all inputs for every output
                    for i,input in enumerate(task2.inputs):
                        #check the type of the slots
                        if(level=="type"):
                            if task2.inputs[i].type==output.type and not used[i]:
                                found=True
                                used[i]=1
                        elif(level=="slot name"):
                            if task2.inputs[i].slot_name==output.slot_name and not used[i]:
                                found=True
                                used[i]=1
                    if not found:
                        return False
                return True
            else:
                return False
        return False

    #remove all the slot names recursively from a tree
    def removeSlotNamesRecursive(self,groupedTask):
        #make object copies of what we need
        cleanedCopy=copy.deepcopy(groupedTask)
        for input in cleanedCopy.inputs:
            input.slot_name=None
        subtasks=[]
        for subtask in cleanedCopy.subtasks:
            subtasks.append(self.removeSlotNamesRecursive(subtask))
        cleanedCopy.subtasks=subtasks
        return cleanedCopy

    #groups the last 2 tasks in the current subtask
    #and also appends new group task to action
    def groupLastTasks(self):
        #get the last 2 tasks
        subtasks=self.tree[self.currentSubtask].subtasks
        #group and keep even the slot names
        groupedTask=(subtasks[-2]).groupWith(subtasks[-1])

        #remove the tasks and add grouped task in
        subtasks=subtasks[:len(subtasks)-2]
        subtasks.append(groupedTask)
        #add the new subtasks list in
        self.tree[self.currentSubtask].subtasks=subtasks 
        print self.tree[self.currentSubtask]
        #reset the input of the top level to be the bottom
        self.tree[self.currentSubtask].inputs=[]
        self.tree[self.currentSubtask].outputs=[]
        for subtask in self.tree[self.currentSubtask].subtasks:
            self.tree[self.currentSubtask].inputs.extend(subtask.inputs)
            self.tree[self.currentSubtask].outputs.extend(subtask.outputs)
        #then lose the slot names to make it a reuseable action
        #clean up the grouped task for use in actions
        cleanedCopy=self.removeSlotNamesRecursive(groupedTask)
        self.actions[groupedTask.name]=cleanedCopy



    #find the correct action and then check if the inputs match
    #if they do, this runs the action and adds it to the current point 
    #in the tree    
    #@return if execution was successful, whether the last two subtasks are group-able, error info
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
                    print "Error matching slot "+input
                    return False,False,{'reason':'match fail','failed_input':input}
                #there is a slot add to input
                else:
                    final_input.append(self.world.getObject(input))

            #execute the task
            success,reason=action.execute(final_input,self.world)
            if not success:
                return False,False,{'reason':reason}
            
            #add the task to the current task that is highlighted-
            self.tree[self.currentSubtask].addSubtask(action)

            #check if the 2 subtasks are groupable
            subtasks=self.tree[self.currentSubtask].subtasks
            isGroupable=False
            if len(subtasks)>1:  
                if self.isGroupable(subtasks[-2],subtasks[-1],"slot name"):
                    isGroupable=True

            return True,isGroupable,None
        else :
            print 'The  %s action does not exist'% (taskName)
            return False,False,{'reason':'The  %s action does not exist'% (taskName)}

    #ending a subtask. Over here we add this task to the complex actions
    def saveCurrentSubtask(self):
        #self.actions.append()
        action= removeSlotNamesRecursive(self.tree[self.currentSubtask])
        self.actions.append(action)

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
        count=0
        output+= "{ \"steps\": [" 
        if not action.groupedSubtasks:
            for subtask in action.subtasks:
                if(count>0):
                    output+=","
                output+=self.build_htn(subtask)
                count+=1
        output+=']}'
        output+=']'
        output+='}'
        return output
 

    '''
        Called when we need to display the HTN
    '''
    def display(self):
        output="{\"data\":["
        #build an outermost task
        for i,action in enumerate(self.tree):
            if not i==0:
                output+="," #reopen the brackets
            output+=self.build_htn(action)

        output+="]}"
        return (output) #TODO this shouldn't be printing but sending to ROS


