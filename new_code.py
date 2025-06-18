#!/usr/bin/env python3
"""
AIMM Robot - ROS2 Implementation with YASMIN State Machine
Converts your existing SMACH implementation to YASMIN
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import threading
import time

# YASMIN imports
import yasmin
from yasmin import State
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

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
# BEHAVIOR TREE NODES (keeping your original structure)
# ============================================================================

class Node:
    def run(self): 
        raise NotImplementedError

class Selector(Node):
    def __init__(self, children): 
        self.children = children
        
    def run(self):
        for c in self.children:
            if c.run(): 
                return True
        return False

class Sequence(Node):
    def __init__(self, children): 
        self.children = children
        
    def run(self):
        for c in self.children:
            if not c.run(): 
                return False
        return True

class Action(Node):
    def __init__(self, fn): 
        self.fn = fn
        
    def run(self): 
        return self.fn()

# ============================================================================
# YASMIN STATES
# ============================================================================

class CalibrationState(State):
    def __init__(self, missions):
        super().__init__(["succeeded", "failed"])
        self.missions = missions
        self.behavior_tree = Sequence([
            Action(self.missions.calibrate), 
            Action(self.missions.wait)
        ])
        
    def execute(self, blackboard):
        try:
            success = self.behavior_tree.run()
            if success:
                return "succeeded"
            else:
                return "failed"
        except Exception as e:
            print(f"Calibration failed: {e}")
            return "failed"

class MissionOneState(State):
    def __init__(self, missions):
        super().__init__(["succeeded", "failed"])
        self.missions = missions
        self.behavior_tree = Sequence([
            Action(self.missions.missionOne), 
            Action(self.missions.wait)
        ])
        
    def execute(self, blackboard):
        try:
            success = self.behavior_tree.run()
            return "succeeded" if success else "failed"
        except Exception as e:
            print(f"Mission One failed: {e}")
            return "failed"

class MissionTwoState(State):
    def __init__(self, missions):
        super().__init__(["succeeded", "failed"])
        self.missions = missions
        self.behavior_tree = Sequence([
            Action(self.missions.missionTwo), 
            Action(self.missions.wait)
        ])
        
    def execute(self, blackboard):
        try:
            success = self.behavior_tree.run()
            return "succeeded" if success else "failed"
        except Exception as e:
            print(f"Mission Two failed: {e}")
            return "failed"

class MissionThreeState(State):
    def __init__(self, missions):
        super().__init__(["succeeded", "failed"])
        self.missions = missions
        self.behavior_tree = Sequence([
            Action(self.missions.missionThree), 
            Action(self.missions.wait)
        ])
        
    def execute(self, blackboard):
        try:
            success = self.behavior_tree.run()
            return "succeeded" if success else "failed"
        except Exception as e:
            print(f"Mission Three failed: {e}")
            return "failed"

class MissionFourState(State):
    def __init__(self, missions):
        super().__init__(["succeeded", "failed"])
        self.missions = missions
        self.behavior_tree = Sequence([
            Action(self.missions.missionFour), 
            Action(self.missions.wait)
        ])
        
    def execute(self, blackboard):
        try:
            success = self.behavior_tree.run()
            return "succeeded" if success else "failed"
        except Exception as e:
            print(f"Mission Four failed: {e}")
            return "failed"

class MissionFiveState(State):
    def __init__(self, missions):
        super().__init__(["succeeded", "failed"])
        self.missions = missions
        self.behavior_tree = Sequence([
            Action(self.missions.missionFive), 
            Action(self.missions.wait)
        ])
        
    def execute(self, blackboard):
        try:
            success = self.behavior_tree.run()
            return "succeeded" if success else "failed"
        except Exception as e:
            print(f"Mission Five failed: {e}")
            return "failed"

class MissionSixState(State):
    def __init__(self, missions):
        super().__init__(["succeeded", "failed"])
        self.missions = missions
        self.behavior_tree = Sequence([
            Action(self.missions.missionSix), 
            Action(self.missions.wait)
        ])
        
    def execute(self, blackboard):
        try:
            success = self.behavior_tree.run()
            return "succeeded" if success else "failed"
        except Exception as e:
            print(f"Mission Six failed: {e}")
            return "failed"

class MissionSevenState(State):
    def __init__(self, missions):
        super().__init__(["succeeded", "failed"])
        self.missions = missions
        self.behavior_tree = Sequence([
            Action(self.missions.missionSeven), 
            Action(self.missions.wait)
        ])
        
    def execute(self, blackboard):
        try:
            success = self.behavior_tree.run()
            return "succeeded" if success else "failed"
        except Exception as e:
            print(f"Mission Seven failed: {e}")
            return "failed"

class MissionEightState(State):
    def __init__(self, missions):
        super().__init__(["succeeded", "failed"])
        self.missions = missions
        self.behavior_tree = Sequence([
            Action(self.missions.missionEight), 
            Action(self.missions.wait)
        ])
        
    def execute(self, blackboard):
        try:
            success = self.behavior_tree.run()
            return "succeeded" if success else "failed"
        except Exception as e:
            print(f"Mission Eight failed: {e}")
            return "failed"

class MissionNineState(State):
    def __init__(self, missions):
        super().__init__(["succeeded", "failed"])
        self.missions = missions
        self.behavior_tree = Sequence([
            Action(self.missions.missionNine), 
            Action(self.missions.wait)
        ])
        
    def execute(self, blackboard):
        try:
            success = self.behavior_tree.run()
            return "succeeded" if success else "failed"
        except Exception as e:
            print(f"Mission Nine failed: {e}")
            return "failed"

class FailSafeState(State):
    def __init__(self, missions):
        super().__init__(["aborted"])
        self.missions = missions
        
    def execute(self, blackboard):
        self.missions.failSafe()
        return "aborted"

class MissionCompleteState(State):
    def __init__(self, missions):
        super().__init__(["finished"])
        self.missions = missions
        
    def execute(self, blackboard):
        self.missions.node.get_logger().info("🎉 All missions completed successfully!")
        return "finished"

# ============================================================================
# MAIN ROS2 NODE WITH YASMIN
# ============================================================================

class AimmRobotNode(Node):
    """Main ROS2 node that runs the AIMM robot with YASMIN"""
    
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
        
        # Create YASMIN state machine
        self.setup_state_machine()
        
        # Set up YASMIN viewer for visualization
        self.yasmin_pub = YasminViewerPub(self, "AIMM_YASMIN", self.sm)
        
        # Start mission execution
        self.mission_thread = threading.Thread(target=self.run_missions)
        self.mission_thread.daemon = True
        self.mission_thread.start()

    def setup_state_machine(self):
        """Create YASMIN state machine"""
        self.sm = StateMachine(outcomes=["mission_complete", "mission_failed"])
        
        # Create blackboard for shared data
        blackboard = yasmin.Blackboard()
        
        # Add states to the state machine
        self.sm.add_state(
            "CALIBRATION",
            CalibrationState(self.missions),
            transitions={
                "succeeded": "MISSION_1",
                "failed": "FAILSAFE"
            }
        )
        
        self.sm.add_state(
            "MISSION_1",
            MissionOneState(self.missions),
            transitions={
                "succeeded": "MISSION_2",
                "failed": "FAILSAFE"
            }
        )
        
        self.sm.add_state(
            "MISSION_2",
            MissionTwoState(self.missions),
            transitions={
                "succeeded": "MISSION_3",
                "failed": "FAILSAFE"
            }
        )
        
        self.sm.add_state(
            "MISSION_3",
            MissionThreeState(self.missions),
            transitions={
                "succeeded": "MISSION_4",
                "failed": "FAILSAFE"
            }
        )
        
        self.sm.add_state(
            "MISSION_4",
            MissionFourState(self.missions),
            transitions={
                "succeeded": "MISSION_5",
                "failed": "FAILSAFE"
            }
        )
        
        self.sm.add_state(
            "MISSION_5",
            MissionFiveState(self.missions),
            transitions={
                "succeeded": "MISSION_6",
                "failed": "FAILSAFE"
            }
        )
        
        self.sm.add_state(
            "MISSION_6",
            MissionSixState(self.missions),
            transitions={
                "succeeded": "MISSION_7",
                "failed": "FAILSAFE"
            }
        )
        
        self.sm.add_state(
            "MISSION_7",
            MissionSevenState(self.missions),
            transitions={
                "succeeded": "MISSION_8",
                "failed": "FAILSAFE"
            }
        )
        
        self.sm.add_state(
            "MISSION_8",
            MissionEightState(self.missions),
            transitions={
                "succeeded": "MISSION_9",
                "failed": "FAILSAFE"
            }
        )
        
        self.sm.add_state(
            "MISSION_9",
            MissionNineState(self.missions),
            transitions={
                "succeeded": "MISSION_COMPLETE",
                "failed": "FAILSAFE"
            }
        )
        
        self.sm.add_state(
            "MISSION_COMPLETE",
            MissionCompleteState(self.missions),
            transitions={
                "finished": "mission_complete"
            }
        )
        
        self.sm.add_state(
            "FAILSAFE",
            FailSafeState(self.missions),
            transitions={
                "aborted": "mission_failed"
            }
        )
        
        # Set initial state
        self.sm.set_start_state("CALIBRATION")

    def run_missions(self):
        """Execute the YASMIN state machine"""
        self.get_logger().info("🚀 Starting AIMM Mission Sequence with YASMIN...")
        
        # Create blackboard for shared data
        blackboard = yasmin.Blackboard()
        
        # Execute state machine
        try:
            outcome = self.sm.execute(blackboard)
            self.get_logger().info(f"✅ Mission sequence completed with outcome: {outcome}")
        except Exception as e:
            self.get_logger().error(f"❌ Mission execution failed: {e}")

    # ROS2 Callbacks
    def buoy_callback(self, msg):
        self.buoy_detected = msg.data
        if msg.data:
            self.get_logger().info("🔍 Buoy detection signal received")

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        if msg.data:
            self.get_logger().info("⚠️ Obstacle detection signal received")

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
