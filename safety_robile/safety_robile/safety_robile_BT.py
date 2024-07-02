#!/usr/bin/env python3

import sys
import operator

import py_trees as pt
import py_trees.console as console
import py_trees_ros as ptr
import rclpy

from safety_robile.behaviors import *


def create_root() -> pt.behaviour.Behaviour:
    """
    Method to structure the behavior tree to monitor battery status and start to rotate if battery is low.
    As well as, to stop if it detects an obstacle in front of it.
    """

    ## define nodes and behaviors ##

    # define root node
    root = pt.composites.Parallel(
        name="root",
        policy=pt.common.ParallelPolicy.SuccessOnAll(
            synchronise=False
        )
    )    

    """
    Create a sequence node called "Topics2BB" and a selector node called "Priorities"    
    """
    topics2BB = pt.composites.Sequence("Topics2BB", memory=False)
    priorities = pt.composites.Selector("Priorities", memory=False)
    
    """
    Using the battery_status2bb class, create a node which subscribes to the topic "/battery_voltage"
    and using the laser_scan_2bb class, create a node which subscribes to the topic "/scan"
    """    
    battery2bb = battery_status2bb(topic_name='/battery_voltage', threshold=20.0)
    LaserScan2BB = laser_scan_2bb(topic_name='/scan', safe_range=0.5)

    """
    Create two sequence nodes named "Battery Emergency" and "Collision Checking"
    """
    battery_emergency = pt.composites.Sequence("Battery Emergency", memory=False)
    collison_emergency = pt.composites.Sequence("Collision Checking", memory=False)

    """
    Using the check_blackboard_variable_value class, create a node called "Battery Low?" which checks if the battery is low,
    as well as a node called "Is Colliding?" to check if the robot is about to collide with an obstacle
    """
    battery_low = pt.behaviours.CheckBlackboardVariableValue(name="Battery Low?", 
                                                             check=pt.common.ComparisonExpression(variable="battery_low_warning",
                                                                                                  value=True,
                                                                                                  operator=operator.eq))


    is_colliding = pt.behaviours.CheckBlackboardVariableValue(name="Is Colliding?", 
                                                             check=pt.common.ComparisonExpression(variable="collison_warning",
                                                                                                  value=True,
                                                                                                  operator=operator.eq))
    
    """
    Using the rotate class, create a node called "Rotate Platform", and using the stop_motion class, create a node called "Stop Platform"
    """
    rotate_platform = rotate(name="Rotate Platform")
    stop_platform = stop_motion(name="Stop Platform")

    """
    Create a node called "Idle", which is a running node to keep the robot idle
    """
    idle = pt.behaviours.Running(name="Idle")

    """
    Construct the behavior tree structure using the nodes and behaviors defined above
    """
    root.add_children([topics2BB, priorities])
    topics2BB.add_children([battery2bb, LaserScan2BB])
    priorities.add_children([collison_emergency, battery_emergency, idle]) 
    battery_emergency.add_children([battery_low, rotate_platform])
    collison_emergency.add_children([is_colliding, stop_platform])

    return root


def main():
    """
    Main function initiates behavior tree construction
    """
    rclpy.init(args=None)
    # Initialising the node with name "behavior_tree"
    root = create_root()
    tree = ptr.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
        )

    # setup the tree
    try:
        tree.setup(timeout=30.0)
    except ptr.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # frequency of ticks
    tree.tick_tock(period_ms=100)    
    
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
