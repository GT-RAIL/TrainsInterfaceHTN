'''
Containers are objects that the robot cannot directly interact with,
but can place objects into
'''
class Container(object):
    def __init__(self, name):
        # A human-reable name
        self.name = name

        # The type of the object (used for substitution)
        self.typeName = "Lunchbox"

        # A list of objects that this container is holding
        self.contains = []

    def addItem(self, item):
        self.contains.append(item)
