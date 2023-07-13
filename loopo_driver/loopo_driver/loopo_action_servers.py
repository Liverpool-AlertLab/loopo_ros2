import rclpy

from loopo_driver import LoopODriver

from rclpy.action import ActionServer
from rclpy.node import Node

from loopo_messages.action import Move, Homing


class ExtensionActionServers(Node):

    def __init__(self, loopo_driver = LoopODriver):
        
        super().__init__('extension_action_servers')
        
        self.gripper = loopo_driver
        
        self.move_action_server = ActionServer(
            self,
            Move,
            'move',
            self.execute_move_callback)
        
        self.home_action_server = ActionServer(
            self,
            Homing,
            'homing',
            self.execute_home_callback)
                    
        self.check_is_ready()
            
        self.get_logger().info('Extension action server started and ready.')

    def execute_move_callback(self, goal_handle):
        
        self.check_is_ready()
        
        self.get_logger().info("Started move callback")
        self.goal = float(goal_handle.request.target)
        self.get_logger().info('Goal received: %f' % (self.goal))
        
        feedback = Move.Feedback()
        result = Move.Result()
                
        self.gripper.send_command(1,3, self.goal)
        self.get_logger().info('position goal command sent.')
                
        self.gripper.send_command(0,0,0.0)
        self.get_logger().info('Status: %d, %d, %d' % (self.gripper.ex_status, self.gripper.ex_control, self.gripper.ex_position))

        feedback.current_position = float(self.gripper.ex_position)
        goal_handle.publish_feedback(feedback)
        
        while self.gripper.ex_status == 3:
            self.gripper.send_command(0,0,0.0)
            feedback.current_position = float(self.gripper.ex_position)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info('Status: %d, %d, %d' % (self.gripper.ex_status, self.gripper.ex_control, self.gripper.ex_position))
            
        if self.gripper.ex_status == 1:
            result.success = True
            result.error = ""   
            goal_handle.succeed()
        elif self.gripper.ex_status == 4:
            result.success = False
            result.error = "Extension hit the endstop"
            goal_handle.abort()
        elif self.gripper.ex_status == 5:
            result.success = False
            result.error = "Extension is stuck"
            self.get_logger().info('Error: %s' % (result.error))
            goal_handle.abort()
                
        self.get_logger().info("Exiting callback")
        return result
    
    def execute_home_callback(self, goal_handle):
        self.get_logger().info("Started homing callback")
        self.goal = float(goal_handle.request.speed)
        self.get_logger().info('Goal received: %f' % (self.goal))
        
        feedback = Homing.Feedback()
        result = Homing.Result()
                
        self.gripper.send_command(1,5, self.goal)
        self.get_logger().info('Homing command sent.')
                
        self.gripper.send_command(0,0,0.0)
        self.get_logger().info('Status: %d, %d, %d' % (self.gripper.ex_status, self.gripper.ex_control, self.gripper.ex_position))


        feedback.current_position = float(self.gripper.ex_position)
        goal_handle.publish_feedback(feedback)
        
        while self.gripper.ex_status == 2:
            self.gripper.send_command(0,0,0.0)
            feedback.current_position = float(self.gripper.ex_position)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info('Status: %d, %d, %d' % (self.gripper.ex_status, self.gripper.ex_control, self.gripper.ex_position))
            
        if self.gripper.ex_status == 1:
            goal_handle.succeed()
            result.success = True
            result.error = ""   
        elif self.gripper.ex_status == 2:
            result.success = False
            result.error = "Extension is homing"
            goal_handle.abort()
        elif self.gripper.ex_status == 4:
            result.success = True
            result.error = "Extension hit the endstop"
            goal_handle.succeed()
        elif self.gripper.ex_status == 5:
            result.success = False
            result.error = "Extension is stuck"
            self.get_logger().info('Error: %s' % (result.error))
            goal_handle.abort()
                
        self.get_logger().info("Exiting callback")
        return result
    
    def check_is_ready(self):
        self.gripper.send_command(0,0,0.0)                    
        if self.gripper.ex_status == 0:
            self.gripper.send_command(1,0,1.0)     
            self.get_logger().info("Extesnion has been enabled") 
        if self.gripper.ex_control != 1:
            self.gripper.send_command(1,1,1.0)
            self.get_logger().info("Extesnion has been put in position control")


            
def main(args=None):
    rclpy.init(args=args)
    
    loopo = LoopODriver(com_port = '/dev/ttyACM1')

    extension_action_server = ExtensionActionServers(loopo_driver=loopo)
    
    rclpy.spin(extension_action_server)
    
    
if __name__ == '__main__':
    main()