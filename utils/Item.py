'''
Items are things the robot can manipulate
'''
class Item(object):
    def __init__(self, name, typeName):
        # A human-reable name for the object
        self.name = name

        # The type of the object (used for substitution)
        self.typeName = typeName

        # A pointer to the lunchbox the item is in or None 
        # if the item is not in a container
        self.inside = None

        # Is the item on the table/Is it possible for the robot 
        # to manipulate it?
        self.manipulable = True

    def putInContainer(self,container):
        self.inside=container
        sellf.manipulable=False

    def removeFromContainer(self):
        self.inside=None
        self.manipulable=True