# --- Behavior Tree Nodes ---
class Node:
    def run(self): raise NotImplementedError

class Selector(Node):
    def __init__(self, children): self.children = children
    def run(self):
        for c in self.children:
            if c.run(): return True # Makes sure at least one of the them succeeds
        return False

class Sequence(Node):
    def __init__(self, children): self.children = children
    def run(self):
        for c in self.children:
            if not c.run(): return False # ALL HAVE TO SUCCEED
        return True

class Action(Node):
    def __init__(self, fn): self.fn = fn
    def run(self): return self.fn()