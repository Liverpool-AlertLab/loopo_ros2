from loopo_gripper.LoopO_driver import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool

from loopo_gripper.srv._send_loop_o_command import SendLoopOCommand

class LoopOInterfaceNode(Node):
    def __init__(self):
        super().__init__('loopo_interface')
        
        self.gripper = loopo_driver()
        update_period = 0.01
        self.update_timer = self.create_timer(update_period,self.update_cb)

        self.ex_endtsop_publisher = self.create_publisher(Bool, 'loopo/extension/endstop', 10)
        self.ex_position_publisher = self.create_publisher(Int32, 'loopo/extension/position', 10)

        self.tw_endtsop_publisher = self.create_publisher(Bool, 'loopo/twist/endstop', 10)
        self.tr_runout_publisher = self.create_publisher(Bool, 'loopo/twist/right/runout', 10)
        self.tl_runout_publisher = self.create_publisher(Bool, 'loopo/twist/left/runout', 10)
        self.tr_position_publisher = self.create_publisher(Int32, 'loopo/twist/right/position', 10)
        self.tl_position_publisher = self.create_publisher(Int32, 'loopo/twist/left/position', 10)

        self.lp_runout_publisher = self.create_publisher(Bool, 'loopo/loop/runout', 10)
        self.force_publisher = self.create_publisher(Float32, 'loopo/loop/force', 10)
        self.lp_position_publisher = self.create_publisher(Int32, 'loopo/loop/position', 10)

        self.command_server = self.create_service(SendLoopOCommand, "loopo/command", self.send_command_cb)

        self.get_logger().info("LoopO Updater node initialised")

    def update_cb(self):
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

        #self.get_logger().info('ex_es: %d - tw_es: %d - tr_ro: %d - tl_ro: %d - lp_fo: %f - ex_po: %d - tr_po: %d - tl_po: %d - lp_po: %d' % (gripper.ex_endstop, gripper.tw_endstop, gripper.tr_runout, gripper.tl_runout, gripper.force, gripper.ex_position, gripper.tr_position, gripper.tl_position, gripper.lp_position))

    def send_command_cb(self, request, response):
        id = request.id
        command = request.command
        value = request.value
        
        sent = response
        self.gripper.send_command(id, command, value)
        sent.sent = True

        return sent
    
def main(args=None):
    rclpy.init(args=args)
    loopo_interface_node = LoopOInterfaceNode()

    rclpy.spin(loopo_interface_node)

    loopo_interface_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()