#!/usr/bin/env python3
"""
AIMM Robot - ROS2 Implementation with SMACC2 State Machine
Converts your existing SMACH implementation to SMACC2
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import threading
import time

# SMACC2 imports
from smacc2_py import *

# ============================================================================
# ROS2 ENHANCED MISSION FUNCTIONS
# ============================================================================

class RosMissionFunctions:
    """Your mission functions converted to use ROS2"""
    
    def __init__(self, node):
        self.node = node
        
    def calibrate(self):
        time_count = 0
        self.node.get_logger().info("Making sure systems are good...")
        
        while time_count <= 2:
            time_count += 1
            time.sleep(1)
            
        if time_count == 3:
            self.node.get_logger().info("Systems Ready!")
            self.publish_status("Systems Ready")
            return True
        else:
            self.node.get_logger().error("Systems NOT Ready")
            return False

    def wait(self):
        self.node.get_logger().info("Awaiting further instruction...")
        self.publish_status("Waiting")
        time.sleep(1)
        return True

    def traverse(self):
        self.node.get_logger().info("Navigating...")
        # Publish movement command to ROS2
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward
        self.node.cmd_vel_pub.publish(cmd)
        time.sleep(2)
        
        # Stop
        cmd.linear.x = 0.0
        self.node.cmd_vel_pub.publish(cmd)
        return True

    def buoyDetected(self):
        if self.node.buoy_detected:
            self.node.get_logger().info("RED and GREEN Buoy detected!")
            self.node.get_logger().info("Navigating in between buoys...")
            self.publish_status("Navigating between buoys")
            return True
        return False

    def failSafe(self):
        self.node.get_logger().warn("FAILSAFE ACTIVATED - Spinny Spin")
        cmd = Twist()
        cmd.angular.z = 1.0  # Spin
        self.node.cmd_vel_pub.publish(cmd)
        time.sleep(3)
        cmd.angular.z = 0.0
        self.node.cmd_vel_pub.publish(cmd)
        return True

    def doge(self):
        self.node.get_logger().info("Object within 5ft...")
        self.traverse()
        self.node.get_logger().info("Fleeted Successfully!")
        return True

    # ========== MISSION IMPLEMENTATIONS ==========
    
    def missionOne(self):
        """Navigate between buoys"""
        self.traverse()
        time.sleep(2)
        
        if self.buoyDetected():
            return True
        return False

    def missionTwo(self):
        """Object avoidance"""
        self.traverse()
        
        if self.node.obstacle_detected:
            self.node.get_logger().info("Object within 5ft...")
            self.traverse()
            self.node.get_logger().info("Fleeted Successfully!")
        return True

    def missionThree(self):
        """Enemy evasion"""
        self.traverse()
        
        self.node.get_logger().info("Enemies Detected VIA RADAR...")
        self.traverse()
        self.node.get_logger().info("Evade Complete!")
        return True

    def missionFour(self):
        """Target interaction"""
        self.traverse()
        
        self.node.get_logger().info("Detected Target!")
        self.node.get_logger().info("Booping Target...")
        time.sleep(2)
        self.node.get_logger().info("Booping Completed!")
        
        self.traverse()
        return True

    def missionFive(self):
        """Deploy collection center"""
        self.traverse()
        
        self.node.get_logger().info("Deploying...")
        time.sleep(3)
        self.node.get_logger().info("Collection Center Dropped...")
        return True

    def missionSix(self):
        """Drone operations"""
        self.traverse()
        self.wait()
        
        self.node.get_logger().info("Communicating to DRONE...")
        self.node.get_logger().info("DRONE launching...")
        time.sleep(5)
        self.node.get_logger().info("DRONE delivered package...")
        self.node.get_logger().info("DRONE returning...")
        time.sleep(3)
        self.node.get_logger().info("DRONE Docked.")
        
        self.traverse()
        return True

    def missionSeven(self):
        """Cargo retrieval"""
        self.traverse()
        
        self.node.get_logger().info("RADAR looking for cargo....")
        self.node.get_logger().info("ZED 2i looking for cargo...")
        time.sleep(3)
        self.node.get_logger().info("CARGO found...")
        
        self.wait()
        self.node.get_logger().info("Scooping CARGO...")
        time.sleep(2)
        self.node.get_logger().info("CARGO retrieved.")
        return True

    def missionEight(self):
        """RASPI communication"""
        self.traverse()
        self.wait()
        
        self.node.get_logger().info("Communicating with RASPI...")
        self.node.get_logger().info("RASPI in DM State...")
        time.sleep(2)
        
        self.traverse()
        return True

    def missionNine(self):
        """Return home"""
        self.node.get_logger().info("Returning home...")
        self.traverse()
        self.node.get_logger().info("Arrived.")
        return True

    def publish_status(self, status):
        """Publish mission status to ROS2 topic"""
        msg = String()
        msg.data = status
        self.node.status_pub.publish(msg)

# ============================================================================
# SMACC2 CLIENT BEHAVIORS
# ============================================================================

class NavigationClient(SmaccClient):
    """Client for navigation operations"""
    def __init__(self):
        super().__init__()
        
    def initialize(self):
        pass

class MissionExecutionClient(SmaccClient):
    """Client for mission execution"""
    def __init__(self):
        super().__init__()
        
    def initialize(self):
        pass

# ============================================================================
# SMACC2 CLIENT BEHAVIORS
# ============================================================================

class CalibrateAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        self.getStateMachine().getGlobalData().missions.calibrate()
        self.postSuccessEvent()

class TraverseAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        self.getStateMachine().getGlobalData().missions.traverse()
        self.postSuccessEvent()

class MissionOneAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        success = self.getStateMachine().getGlobalData().missions.missionOne()
        if success:
            self.postSuccessEvent()
        else:
            self.postFailureEvent()

class MissionTwoAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        success = self.getStateMachine().getGlobalData().missions.missionTwo()
        if success:
            self.postSuccessEvent()
        else:
            self.postFailureEvent()

class MissionThreeAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        success = self.getStateMachine().getGlobalData().missions.missionThree()
        if success:
            self.postSuccessEvent()
        else:
            self.postFailureEvent()

class MissionFourAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        success = self.getStateMachine().getGlobalData().missions.missionFour()
        if success:
            self.postSuccessEvent()
        else:
            self.postFailureEvent()

class MissionFiveAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        success = self.getStateMachine().getGlobalData().missions.missionFive()
        if success:
            self.postSuccessEvent()
        else:
            self.postFailureEvent()

class MissionSixAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        success = self.getStateMachine().getGlobalData().missions.missionSix()
        if success:
            self.postSuccessEvent()
        else:
            self.postFailureEvent()

class MissionSevenAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        success = self.getStateMachine().getGlobalData().missions.missionSeven()
        if success:
            self.postSuccessEvent()
        else:
            self.postFailureEvent()

class MissionEightAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        success = self.getStateMachine().getGlobalData().missions.missionEight()
        if success:
            self.postSuccessEvent()
        else:
            self.postFailureEvent()

class MissionNineAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        success = self.getStateMachine().getGlobalData().missions.missionNine()
        if success:
            self.postSuccessEvent()
        else:
            self.postFailureEvent()

class WaitAction(SmaccClientBehavior):
    def __init__(self):
        super().__init__()
        
    def onEntry(self):
        self.getStateMachine().getGlobalData().missions.wait()
        self.postSuccessEvent()

# ============================================================================
# SMACC2 STATES
# ============================================================================

class StCalibration(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        self.addTransition(EvCbSuccess(CalibrateAction), StMissionOne)
        self.addTransition(EvCbFailure(CalibrateAction), StFailSafe)
        
    def onEntry(self):
        self.requiresClient(MissionExecutionClient)
        self.requiresComponent(CalibrateAction)

class StMissionOne(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        self.addTransition(EvCbSuccess(MissionOneAction), StMissionTwo)
        self.addTransition(EvCbFailure(MissionOneAction), StFailSafe)
        
    def onEntry(self):
        self.requiresClient(MissionExecutionClient)
        self.requiresComponent(MissionOneAction)

class StMissionTwo(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        self.addTransition(EvCbSuccess(MissionTwoAction), StMissionThree)
        self.addTransition(EvCbFailure(MissionTwoAction), StFailSafe)
        
    def onEntry(self):
        self.requiresClient(MissionExecutionClient)
        self.requiresComponent(MissionTwoAction)

class StMissionThree(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        self.addTransition(EvCbSuccess(MissionThreeAction), StMissionFour)
        self.addTransition(EvCbFailure(MissionThreeAction), StFailSafe)
        
    def onEntry(self):
        self.requiresClient(MissionExecutionClient)
        self.requiresComponent(MissionThreeAction)

class StMissionFour(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        self.addTransition(EvCbSuccess(MissionFourAction), StMissionFive)
        self.addTransition(EvCbFailure(MissionFourAction), StFailSafe)
        
    def onEntry(self):
        self.requiresClient(MissionExecutionClient)
        self.requiresComponent(MissionFourAction)

class StMissionFive(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        self.addTransition(EvCbSuccess(MissionFiveAction), StMissionSix)
        self.addTransition(EvCbFailure(MissionFiveAction), StFailSafe)
        
    def onEntry(self):
        self.requiresClient(MissionExecutionClient)
        self.requiresComponent(MissionFiveAction)

class StMissionSix(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        self.addTransition(EvCbSuccess(MissionSixAction), StMissionSeven)
        self.addTransition(EvCbFailure(MissionSixAction), StFailSafe)
        
    def onEntry(self):
        self.requiresClient(MissionExecutionClient)
        self.requiresComponent(MissionSixAction)

class StMissionSeven(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        self.addTransition(EvCbSuccess(MissionSevenAction), StMissionEight)
        self.addTransition(EvCbFailure(MissionSevenAction), StFailSafe)
        
    def onEntry(self):
        self.requiresClient(MissionExecutionClient)
        self.requiresComponent(MissionSevenAction)

class StMissionEight(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        self.addTransition(EvCbSuccess(MissionEightAction), StMissionNine)
        self.addTransition(EvCbFailure(MissionEightAction), StFailSafe)
        
    def onEntry(self):
        self.requiresClient(MissionExecutionClient)
        self.requiresComponent(MissionEightAction)

class StMissionNine(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        self.addTransition(EvCbSuccess(MissionNineAction), StMissionComplete)
        self.addTransition(EvCbFailure(MissionNineAction), StFailSafe)
        
    def onEntry(self):
        self.requiresClient(MissionExecutionClient)
        self.requiresComponent(MissionNineAction)

class StMissionComplete(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        pass
        
    def onEntry(self):
        self.getLogger().info("All missions completed successfully!")

class StFailSafe(SmaccState):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        pass
        
    def onEntry(self):
        self.getStateMachine().getGlobalData().missions.failSafe()
        self.getLogger().error("Mission failed - failsafe activated")

# ============================================================================
# MAIN SMACC2 STATE MACHINE
# ============================================================================

class AimmStateMachine(SmaccStateMachineBase):
    def __init__(self):
        super().__init__()
        
    def configure(self):
        # Add global data
        self.setGlobalData("missions", None)
        
        # Configure state machine with initial state
        self.addState(StCalibration, initial_state=True)
        self.addState(StMissionOne)
        self.addState(StMissionTwo)
        self.addState(StMissionThree)
        self.addState(StMissionFour)
        self.addState(StMissionFive)
        self.addState(StMissionSix)
        self.addState(StMissionSeven)
        self.addState(StMissionEight)
        self.addState(StMissionNine)
        self.addState(StMissionComplete)
        self.addState(StFailSafe)

# ============================================================================
# MAIN ROS2 NODE
# ============================================================================

class AimmRobotNode(Node):
    """Main ROS2 node that runs the AIMM robot with SMACC2"""
    
    def __init__(self):
        super().__init__('aimm_robot')
        
        # Initialize mission functions
        self.missions = RosMissionFunctions(self)
        
        # ROS2 Publishers
        self.status_pub = self.create_publisher(String, 'mission_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # ROS2 Subscribers
        self.buoy_sub = self.create_subscription(
            Bool, 'buoy_detected', self.buoy_callback, 10)
        self.obstacle_sub = self.create_subscription(
            Bool, 'obstacle_detected', self.obstacle_callback, 10)
        
        # State variables
        self.buoy_detected = False
        self.obstacle_detected = False
        
        # Create and start SMACC2 state machine
        self.sm = AimmStateMachine()
        self.sm.setGlobalData("missions", self.missions)
        
        # Start state machine in separate thread
        self.sm_thread = threading.Thread(target=self.run_state_machine)
        self.sm_thread.daemon = True
        self.sm_thread.start()

    def run_state_machine(self):
        """Execute the SMACC2 state machine"""
        self.get_logger().info("Starting AIMM Mission Sequence with SMACC2...")
        self.sm.execute()

    # ROS2 Callbacks
    def buoy_callback(self, msg):
        self.buoy_detected = msg.data
        if msg.data:
            self.get_logger().info("Buoy detection signal received")

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        if msg.data:
            self.get_logger().info("Obstacle detection signal received")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AimmRobotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
