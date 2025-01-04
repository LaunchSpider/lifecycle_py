#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from custom_interfaces.action import PositionVelocity
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class MoveRobotServerNode(LifecycleNode):
    def __init__(self):
        super().__init__("move_robot_server")
        self.goal_lock_ = threading.Lock()
        self.goal_handle_: ServerGoalHandle = None
        self.robot_position_ = 50
        self.server_activated_ = False
        self.get_logger().info("Robot position: " + str(self.robot_position_))
        
    def on_configure(self, previous_state: LifecycleState):
        self.declare_parameter("robot_name", rclpy.Parameter.Type.STRING)
        self.robot_name_ = self.get_parameter("robot_name").value
        self.move_robot_server_ = ActionServer(
            self,
            PositionVelocity,
            "move_robot_" + self.robot_name_,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action server has been started")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, previous_state: LifecycleState):
        self.undeclare_parameter("robot_name")
        self.robot_name_ = ""
        self.move_robot_server_.destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("Activate node")
        self.server_activated_ = True
        return super().on_activate(previous_state)
    
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("Deactivate node")
        self.server_activated_ = False
        # New goal is valid, abort previous goal and accept new goal
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.goal_handle_.abort()
        return super().on_deactivate(previous_state)
    
    def on_shutdown(self, previous_state: LifecycleState):
        self.undeclare_parameter("robot_name")
        self.robot_name_ = ""
        self.move_robot_server_.destroy()
        return TransitionCallbackReturn.SUCCESS

    def goal_callback(self, goal_request: PositionVelocity.Goal):
        self.get_logger().info("Received a new goal")
        
        if not self.server_activated_:
            self.get_logger().warn("Node not activated yet")
            return GoalResponse.REJECT

        if goal_request.position not in range(0, 100) or goal_request.velocity <= 0:
            self.get_logger().warn("Invalid position/velocity, reject goal")
            return GoalResponse.REJECT
        
        self.get_logger().info("Accept goal")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Received a cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        goal_position = goal_handle.request.position
        velocity = goal_handle.request.velocity

        result = PositionVelocity.Result()
        feedback = PositionVelocity.Feedback()

        self.get_logger().info("Execute goal")
        while rclpy.ok():
            if not goal_handle.is_active:
                result.position = self.robot_position_
                result.message = "Preempted by another goal, or node deactivated"
                return result
            
            if goal_handle.is_cancel_requested:
                result.position = self.robot_position_
                if goal_position == self.robot_position_:
                    result.message = "Success after cancel request"
                    goal_handle.succeed()
                else:
                    result.message = "Canceled"
                    goal_handle.canceled()
                return result

            diff = goal_position - self.robot_position_

            if diff == 0:
                result.position = self.robot_position_
                result.message = "Success"
                goal_handle.succeed()
                return result
            elif diff > 0:
                if diff >= velocity:
                    self.robot_position_ += velocity
                else:
                    self.robot_position_ += diff
            else:
                if abs(diff) >= velocity:
                    self.robot_position_ -= velocity
                else:
                    self.robot_position_ -= abs(diff)

            self.get_logger().info("Robot position: " + str(self.robot_position_))
            feedback.current_position = self.robot_position_
            goal_handle.publish_feedback(feedback)

            time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()