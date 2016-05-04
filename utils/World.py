#!/usr/bin/env python
"""
Utility class called World. It stores the current state of the Robots and also which types
of which Items are available
"""
__author__ =  'Aaron St. Clair <astclair@cc.gatech.edu>'
__version__=  '0.1'
__license__ = 'BSD'

from Container import Container
from Item import Item
from Action import Slot

class World(object):
    def __init__(self,items):
        # Item specification for initialization 
        self.itemSpec = items['items']

        # A list of all items possible
        self.items = []

        # A list of all containers (assumed to be non-manipulable)
        self.numContainers = 2
        self.containers = []

        # Initialize containers
        for container in items['containers']:
            self.containers.append( Container(container['name']) )

        # Intialize items
        for i in self.itemSpec:
            self.items.append( Item(i['name'], i['type']) )

        #gets the current list of available items
        self.available_items=[]

        #gets the item that the robot is holding
        self.holding=None

    '''
    Used for getting an object in the item list 
    '''
    def getObject(self,input):
        for item in self.available_items:
            if self.compare(input,item.name):
                return item

        for container in self.containers:
            if(input==container.name):
                return container


    '''
    Comparing 2 items should be compared like this 
    to avoid any issues with typos and strings with
    numeric IDs
    '''
    def compare(self,item1,item2):
        if item1.upper().startswith(item2.upper()):
            return True
        else:
            return False


    def getCurrentWorldState(self):
        output={}
        output['available_items']=[]
        for item in self.available_items:
            output['available_items'].append({'name':item.name,'type':item.typeName,'manipulable':item.manipulable})

        output['containers']=[]
        for container in self.containers:
            output['containers'].append({'name':container.name,'items':[item.name for item in container.contains]})
        
        if(self.holding==None):
            output['robot_holding']=True
        else:
            output['robot_holding']=self.holding.name
        return output

    '''
    This is called when we get a new set of recognized items
    We reset which items are considererd manipulable at that point
    '''
    def refreshItems(self,items):
        self.available_items=[]
        for recognized_item in items:
            for item in self.items:
                # At this point o will have a numeric id at the end of its name if
                # there are multiple versions of this object.
                if self.compare(recognized_item.name,item.name):
                    #This item is a match so mark it as recognized
                    self.available_items.append(item)

    '''
    get objects by type
    Criterion is a function that checks for any extra criteria you might need. 
    Pass a function to it
    '''
    def getObjectsByType(self,type,criterion=None):
        objects=[]
        if criterion==None:
            criterion= lambda world,input : True
        if(type.lower()=='item'):
            for item in self.items:
                if item.manipulable:
                    if(criterion(self,item.name)):
                        objects.append(item.name)
        elif(type.lower()=='container'):
            for box in self.containers:
                if(criterion(self,box)):
                    objects.append(box.name)
        return objects

    '''
    Find an alternative for all the items. Take an input and by types 
    check if we have any alternatives
    '''
    def findAlternatives(self,input):
        type=''
        alternatives=[]
        for item in self.items:
            if input==item.name:
                type=item.typeName
        for item in self.available_items:
            if type==item.typeName and not item.name ==input:
                    alternatives.append(item.name)
        alternatives.append('None. Undo! ')
        return alternatives


    #make a slot class which is specified down to the input
    # @return whether or not making a slot is possible
    def makeSlot(self,input,action_inputs):
        #check if it is an item or container
        for item in self.available_items:
            if self.compare(input,item.name):
                for action_input in action_inputs:
                    if action_input.slot_name==None and action_input.type=='Item':
                        action_input.slot_name=input
                        return True

        for container in self.containers:
            if(input==container.name):
                for action_input in action_inputs:
                    if action_input.slot_name==None and action_input.type=='Container':
                        action_input.slot_name=input
                        return True
        return False
