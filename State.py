# --- FSM ---
class State:
    def __init__(self, name, behavior_tree):
        self.name = name
        self.behavior_tree = behavior_tree

    def on_enter(self):
        print(f"[FSM] Enter {self.name}")

    def on_exit(self):
        print(f"[FSM] Exit {self.name}")

    def run(self):
        print(f"[FSM] Running {self.name}...")
        self.behavior_tree.run()