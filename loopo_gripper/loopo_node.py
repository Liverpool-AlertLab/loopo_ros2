from loopo_gripper.LoopO_driver import *
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32, Bool

class LoopOnode(Node):

    def __init__(self):
        super().__init__('loopo_node')
        self.gripper = loopo_driver()
        self.ex_endtsop_publisher = self.create_publisher(Bool, 'loopo_gripper/ex_endstop', 10)
        self.tw_endtsop_publisher = self.create_publisher(Bool, 'loopo_gripper/tw_endstop', 10)
        self.tr_runout_publisher = self.create_publisher(Bool, 'loopo_gripper/tr_endstop', 10)
        self.tl_runout_publisher = self.create_publisher(Bool, 'loopo_gripper/tl_endstop', 10)
        self.lp_runout_publisher = self.create_publisher(Bool, 'loopo_gripper/lp_endstop', 10)
        self.force_publisher = self.create_publisher(Float32, 'loopo_gripper/force', 10)
        self.ex_position_publisher = self.create_publisher(Int32, 'loopo_gripper/ex_position', 10)
        self.tr_position_publisher = self.create_publisher(Int32, 'loopo_gripper/tr_position', 10)
        self.tl_position_publisher = self.create_publisher(Int32, 'loopo_gripper/tl_position', 10)
        self.lp_position_publisher = self.create_publisher(Int32, 'loopo_gripper/lp_position', 10)
        timer_period = 0.1
        update_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.update_timer = self.create_timer(update_period,self.update_callback)


    def timer_callback(self):
        self.gripper.update()
        ex_endstop = Bool()
        tw_endstop = Bool()
        tr_runout = Bool()
        tl_runout = Bool()
        lp_runout = Bool()
        force = Float32()
        ex_position = Int32()
        tr_position = Int32()
        tl_position = Int32()
        lp_position = Int32()

        ex_endstop.data = self.gripper.ex_endstop
        tw_endstop.data = self.gripper.tw_endstop
        tr_runout.data = self.gripper.tr_runout
        tl_runout.data = self.gripper.tl_runout
        lp_runout.data = self.gripper.lp_runout
        force.data = self.gripper.force
        ex_position.data = self.gripper.ex_position
        tr_position.data = self.gripper.tr_position
        tl_position.data = self.gripper.tl_position
        lp_position.data = self.gripper.lp_position

        self.ex_endtsop_publisher.publish(ex_endstop)
        self.tw_endtsop_publisher.publish(tw_endstop)
        self.tr_runout_publisher.publish(tr_runout) 
        self.tl_runout_publisher.publish(tl_runout) 
        self.lp_runout_publisher.publish(lp_runout)
        self.force_publisher.publish(force)
        self.ex_position_publisher.publish(ex_position)
        self.tr_position_publisher.publish(tl_position)
        self.tl_position_publisher.publish(tl_position)
        self.lp_position_publisher.publish(lp_position)

    def update_callback(self):
        self.gripper.update()

def main(args=None):
    rclpy.init(args=args)

    loopo_node = LoopOnode()

    rclpy.spin(loopo_node)

    loopo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()