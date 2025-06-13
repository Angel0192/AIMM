def calibrate():
    time = 0
    print("Making sure systems are good...")
    
    while time <= 2:
        time += 1
    if time == 3:
        print("Systems Ready!")
    else:
        print("Systems NOT Ready")
        

def wait():
    print("Awaiting further instruction...")

def traverse():
    print("Navigating...")

def buoyDetected():
    print("RED and GREEN Buoy detected!")
    print("Navigating in between buoys...")
    
def failSafe():
    print("Spinny Spin")

def doge():
    print("Object within 5ft...")
    traverse
    print("Fleeted Successfully!")
        

# Mission One

def missionOne():
    time = 0
    
    traverse()
    
    while time < 2:
        time += 1
    
    if time == 2:
        buoyDetected() 
        
# Mission Two

def missionTwo():
    traverse
    
    print("Object within 5ft...")
    traverse
    print("Fleeted Successfully!")

# Mission Three
def missionThree():
    traverse()
    
    print("Enemies Detected VIA RADAR...")
    
    traverse
    
    print("Evade Complete!")

# Mission Four

def missionFour():
    traverse
    
    print("Detected Target!")
    print("Booping Target...")
    print("Booping Completed!")
    
    traverse

# Mission Five

def missionFive():
    traverse
    
    print("Deploying...")
    print("Collection Center Dropped...")

# Mission Six

def missionSix():
    traverse
    wait
    print("Communicating to DRONE...")
    print("DRONE launching...")
    print("DRONE delivered package...")
    print("DRONE returning...")
    print("DRONE Docked.")
    traverse

# Mission Seven
def missionSeven():
    traverse
    print("RADAR looking for cargo....")
    print("ZED 2i looking for cargo...")
    print("CARGO found...")
    wait
    print("Scooping CARGO...")
    print("CARGO retrieved.")

# Mission Eight
def missionEight():
    traverse
    wait
    print("Communicating with RASPI...")
    print("RASPI in DM State...")
    traverse

# Mission Nine
def missionNine():
    print("Returning home...")
    traverse
    print("Arrived.")
