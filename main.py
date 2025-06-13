from constants import *
from FSM import *
from State import *
from Selectors import *

# --- Define Behavior Trees ---
# This takes in the sequence of actions that can be modified in constants.py | Sequence: ALL have to be true - Selecitor: ONE can be true

calibrate_bt = Sequence([Action(calibrate), Action(wait)])
mOne_bt = Sequence([Action(missionOne), Action(wait)])
mTwo_bt = Sequence([Action(missionTwo),Action(wait)])
mThree_bt = Sequence([Action(missionThree), Action(wait)])
mFour_bt = Sequence([Action(missionFour), Action(wait)])
mFive_bt = Sequence([Action(missionFive), Action(wait)])
mSix_bt = Sequence([Action(missionSix), Action(wait)])
mSeven_bt = Sequence([Action(missionSeven), Action(wait)])
mEight_bt = Sequence([Action(missionEight), Action(wait)])
mNine_bt = Sequence([Action(missionNine), Action(wait)])

# --- Define FSM States ---

calibration = State("calibration", calibrate_bt)
missOne = State("Mission 1", mOne_bt)
missTwo = State("Mission 2", mTwo_bt)
missThree = State("Mission 3", mThree_bt)
missFour = State("Mission 4", mFour_bt)
missFive = State("Mission 5", mFive_bt)
missSix = State("Mission 6", mSix_bt)
missSeven = State("Mission 7", mSeven_bt)
missEight = State("Mission 8", mEight_bt)
missNine = State("Mission 9", mNine_bt)

# --- FSM Setup ---
fsm = FSM(calibration)
fsm.add_transition(calibration, "M1", missOne)
fsm.add_transition(missOne, "M2", missTwo)
fsm.add_transition(missTwo, "M3", missThree)
fsm.add_transition(missThree,"M4", missFour)
fsm.add_transition(missFour, "M5", missFive)
fsm.add_transition(missFive, "M6", missSix)
fsm.add_transition(missSix, "M7", missSeven)
fsm.add_transition(missSeven, "M8", missEight)
fsm.add_transition(missEight, "M9", missNine)

# --- Simulate ---
fsm.update() 
fsm.handle_event("M1")
fsm.update()
fsm.handle_event("M2")
fsm.update()
fsm.handle_event("M3")
fsm.update()
fsm.handle_event("M4")
fsm.update()
fsm.handle_event("M5")
fsm.update()
fsm.handle_event("M6")
fsm.update()
fsm.handle_event("M7")
fsm.update()
fsm.handle_event("M8")
fsm.update()
fsm.handle_event("M9")
fsm.update()