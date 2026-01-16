#!/usr/bin/env python3
import time
import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse , CancelResponse
from rclpy.action.server import ServerGoalHandle
from my_robot_interfaces.action import MoveRobot
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class MoveRobotServerNode(Node): # MODIFY NAME

    def __init__(self):
        super().__init__("move_robot_server") # MODIFY NAME
        self.goal_handle_: ServerGoalHandle =  None
        self.goal_lock_ = threading.Lock()
        self.robot_position_=0
        self.count_until_server = ActionServer(
            self,
            MoveRobot,
            "move_robot",
            goal_callback=self.goal_callback,
            execute_callback=self.execute_callback,
            callback_group = ReentrantCallbackGroup()
            )
        self.get_logger().info("Action server has been started")
    
    # def cancel_callback(self, goal_handle: ServerGoalHandle):
    #     self.get_logger().info("Recived a cancel request")
    #     return CancelResponse.ACCEPT # or REJECT
        
    def goal_callback(self, goal_request: MoveRobot.Goal):
        self.get_logger().info("Recived a goal")

        # Policy : refuse new goal if curengt goal is active
        #####-there is a  with self.goal_lock_: <- missed here  but code worked without it
        # if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #     self.get_logger().info("A goal is already active rejecting new goal")
        #     return GoalResponse.REJECT

        #Validate the goal request
        if goal_request.position <0 or goal_request.position >100:
            self.get_logger().info("Rejecting the goal")
            return GoalResponse.REJECT

        # prempt existing goal once new goal recived
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().info("Abort current goal and accept new")
                self.goal_handle_.abort()

        # self.get_logger().info("Accepting the goal")
        return GoalResponse.ACCEPT

    # def handle_accepted_callback(self ,  goal_handle:ServerGoalHandle):
    #     with self.goal_lock_:
    #         if self.goal_handle_ is not None:
    #             self.goal_que_.append(goal_handle)
    #         else:
    #             goal_handle.execute()

    def execute_callback(self, goal_handle:ServerGoalHandle):

        with self.goal_lock_:
            self.goal_handle_=goal_handle

        position =  goal_handle.request.position
        velocity = goal_handle.request.velocity

        #Execute the action
        self.get_logger().info("Executing the goal")
        
        feedback = MoveRobot.Feedback()
        result = MoveRobot.Result()
        current_position=self.robot_position_

        self.get_logger().info(f"Moving from {current_position} to position {position} with velocity {velocity}")
        # if(position):
        #     current_position=position
        # else:
        #     counter=0

        steps=0
        if position>=current_position:
            steps=int((position-current_position)//velocity)
        else:
            steps=int((current_position-position)//velocity)
            velocity=-velocity

        
        for i in range(steps):

            if not goal_handle.is_active:
                result.position = int(current_position)
                self.robot_position_=int(current_position)
                result.message = "Preempted by another goal"
                return result

            
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Cancelling the goal")
                goal_handle.canceled()
                result.position = int(current_position)
                result.message = "Goal Canceled"
                self.process_next_goal_in_queue()
                return result
            feedback.current_position = int(current_position)
            goal_handle.publish_feedback(feedback)

            current_position+=velocity
            self.get_logger().info(str(current_position))
            
            time.sleep(1/abs(velocity))


        remainder=position-current_position
        if(remainder!=0):
            current_position+=remainder
            self.get_logger().info(str(current_position))
            feedback.current_position = int(current_position)
            goal_handle.publish_feedback(feedback)
            time.sleep(1/abs(remainder))
        self.robot_position_=int(current_position)
        #Once goal done set final state
        goal_handle.succeed()

        #and send the result
        result = MoveRobot.Result()
        result.position=int(current_position)
        result.message="Goal Succeeded"
        # self.process_next_goal_in_queue()
        return result
    # def process_next_goal_in_queue(self):
    #     with self.goal_lock_:
    #         if len(self.goal_que_)>0:
    #             self.goal_que_.pop(0).execute()
    #         else:
    #             self.goal_handle_=None
def main(args=None):
    rclpy.init(args=args)
    node = MoveRobotServerNode() # MODIFY NAME
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()