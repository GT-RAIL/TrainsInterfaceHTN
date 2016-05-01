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
        self.itemSpec = items['item']

        # A list of all items possible
        self.items = []

        # A list of all containers (assumed to be non-manipulable)
        self.numContainers = 2
        self.containers = []

        # Initialize containers
        for container in items['containers']
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
    def getObject(self,item):
        for item in self.available_items:
            if input==item.name and item.manipulable:
                return item

        for container in self.containers:
            if(input==container.name):
                return container


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
                if recognized_item['name'].upper().startswith(item.name.upper()):
                    #This item is a match so mark it as recognized
                    self.available_items.append(item)


    #make a slot class which is specified down to the input
    # @return whether or not making a slot is possible
    def makeSlot(self,input,action_inputs):
        #check if it is an item or container
        for item in self.available_items:
            if input==item.name and item.manipulable:
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
