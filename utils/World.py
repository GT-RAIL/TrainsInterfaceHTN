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
    def __init__(self):
        # TODO: read this from file
        # Item specification for initialization 
        self.itemSpec = [
            {"name": "Apple", "type": "Fruit"}, 
            {"name": "Cheezits", "type": "Snack"},
            {"name": "Coffee", "type": "Drink"},
            {"name": "Cookies", "type": "Snack"},
            {"name": "Juice", "type": "Drink"}, 
            {"name": "Lemon", "type": "Fruit"}, 
            {"name": "Milk", "type": "Drink"}, 
            {"name": "Nutella", "type": "Snack"}, 
            {"name": "Orange", "type": "Fruit"}, 
            {"name": "Peach", "type": "Fruit"}, 
            {"name": "Pear", "type": "Fruit"}, 
            {"name": "Raisins", "type": "Snack"}, 
            {"name": "Soup", "type": "Main"}, 
            {"name": "Tuna", "type": "Main"}]

        # A list of all items
        self.items = []

        # A list of all containers (assumed to be non-manipulable)
        self.numContainers = 2
        self.containers = []

        # Initialize containers
        for i in range(0, self.numContainers):
            self.containers.append( Container("Lunchbox " + str(i+1)) )

        # Intialize items
        for i in self.itemSpec:
            self.items.append( Item(i['name'], i['type']) )

        #gets the item that the robot is holding
        self.holding=None


    #make a slot clas which is specified down to the input
    # @return whether or not making a slot is possible
    def makeSlot(self,input,action_inputs):
        #check if it is an item or container
        for item in self.items:
            if(input==item.name):
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
