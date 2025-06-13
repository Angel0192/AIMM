class FSM:
    def __init__(self, initial_state):
        self.current_state = initial_state
        self.current_state.on_enter()
        self.transitions = {}

    def add_transition(self, from_state, event, to_state):
        self.transitions[(from_state.name, event)] = to_state

    def handle_event(self, event):
        key = (self.current_state.name, event)
        if key in self.transitions:
            self.current_state.on_exit()
            self.current_state = self.transitions[key]
            self.current_state.on_enter()
        else:
            print(f"No transition from {self.current_state.name} on {event}")

    def update(self):
        self.current_state.run()