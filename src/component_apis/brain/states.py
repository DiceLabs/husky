class StateMachine:
    def __init__(self, initialState):
        self.currentState = initialState
        self.currentState.run()

class State:
    def run(self):
        assert 0, "run not implemented"
    def next(self, input):
        assert 0, "next not implemented"



class StateT(State):
    def __init__(self):
        self.transitions = None
    def next(self, input):
        if self.transitions.has_key(input):
            return self.transitions[input]
        else:
            raise "Input not supported for current state"

class Waiting(StateT):
    def run(self):
        print("Waiting: Broadcasting cheese smell")
    def next(self, input):
        # Lazy initialization:
        if not self.transitions:
            self.transitions = {
              MouseAction.appears : MouseTrap.luring
            }
        return StateT.next(self, input)

class Luring(StateT):
    def run(self):
        print("Luring: Presenting Cheese, door open")
    def next(self, input):
        # Lazy initialization:
        if not self.transitions:
            self.transitions = {
              MouseAction.enters : MouseTrap.trapping,
              MouseAction.runsAway : MouseTrap.waiting
            }
        return StateT.next(self, input)

class Trapping(StateT):
    def run(self):
        print("Trapping: Closing door")
    def next(self, input):
        # Lazy initialization:
        if not self.transitions:
            self.transitions = {
              MouseAction.escapes : MouseTrap.waiting,
              MouseAction.trapped : MouseTrap.holding
            }
        return StateT.next(self, input)

class Holding(StateT):
    def run(self):
        print("Holding: Mouse caught")
    def next(self, input):
        # Lazy initialization:
        if not self.transitions:
            self.transitions = {
              MouseAction.removed : MouseTrap.waiting
            }
        return StateT.next(self, input)

class MouseTrap(StateMachine):
    def __init__(self):
        # Initial state
        StateMachine.__init__(self, MouseTrap.waiting)

# Static variable initialization:
MouseTrap.waiting = Waiting()
MouseTrap.luring = Luring()
MouseTrap.trapping = Trapping()
MouseTrap.holding = Holding()

moves = map(string.strip,
  open("../mouse/MouseMoves.txt").readlines())
mouseMoves = map(MouseAction, moves)
MouseTrap().runAll(mouseMoves)








class Brain():
    def __init__():
