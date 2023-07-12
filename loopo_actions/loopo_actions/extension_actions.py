import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from time import sleep

from loopo_messages.action import Move
from loopo_messages.srv import SendCommand


class ExtensionActionServers(Node):

    def __init__(self):
        
        super().__init__('extension_action_servers')
        
        self.command = SendCommand.Request()
        
        self._action_server = ActionServer(
            self,
            Move,
            'move',
            self.execute_move_callback)
        
        self.cli = self.create_client(SendCommand, 'loopo_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        self.check_is_ready()
            
        self.get_logger().info('Extension action server started and ready.')

    def execute_move_callback(self, goal_handle):
        
        self.get_logger().info("Started move callback")
        self.goal = float(goal_handle.request.target)
        self.get_logger().info('Goal received: %f' % (self.goal))
        
        feedback = Move.Feedback()
        result = Move.Result()
                
        status = self.send_command(1,3, self.goal)
        self.get_logger().info('position goal command sent.')
                
        self.send_command(0,0,0.0)
        status = self.send_command(0,0,0.0)
        self.get_logger().info('Status: %d, %d, %d' % (status.ex_status, status.ex_control, status.ex_position))

        feedback.current_position = float(status.ex_position)
        goal_handle.publish_feedback(feedback)
        
        while status.ex_status != 1:
            
            if status.ex_status == 2:
                result.success = False
                result.error = "Extension is homing"
                goal_handle.abort()
                return result
            elif status.ex_status == 4:
                result.success = False
                result.error = "Extension hit the endstop"
                goal_handle.abort()
                return result
            elif status.ex_status == 5:
                result.success = False
                result.error = "Extension is stuck"
                self.get_logger().info('Error: %s' % (result.error))
                goal_handle.abort()
                return result
                
            status = self.send_command(0,0,0.0)
            feedback.current_position = float(status.ex_position)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info('Status: %d, %d, %d' % (status.ex_status, status.ex_control, status.ex_position))
        
        goal_handle.succeed()
        result.success = True
        result.error = ""
        self.get_logger().info("Exiting callback")
        return result
    
    def send_command(self, id, command, value):
        self.command.id = id
        self.command.command = command
        self.command.value = value
        self.future = self.cli.call_async(self.command)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def check_is_ready(self):
        status = self.send_command(0,0,0.0)                    
        if status.ex_status == 0:
            status = self.send_command(1,0,1.0)      
        if status.ex_control != 1:
            status = self.send_command(1,1,1.0)


def main(args=None):
    rclpy.init(args=args)

    extension_action_server = ExtensionActionServers()
    
    rclpy.spin(extension_action_server)


if __name__ == '__main__':
    main()