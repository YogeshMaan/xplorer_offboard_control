import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry, VehicleStatus
import math


class PulleyPull(Node):
    "Node for external wrench calculations by pulling the weight using pull"

    def __init__(self) -> None:
        super().__init__('pulley_pull')

        #Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/offboard_control_mode/in', 10)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/trajectory_setpoint/in', 10)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/vehicle_command/in', 10)
        
        #Create subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, 'fmu/vehicle_odometry/out',self.vehicle_odometry_callback, 10)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/vehicle_status/out', self.vehicle_status_callback, 10
        )

        #Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.waypoints = self.generate_waypoints()
        self.wp_num = 0
        self.err = 10 #initialise as high value
        self.thres_err = .15



        #create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def generate_waypoints(self):
        #generate waypoints for a rectangular trajectory
        self.get_logger().info("----Generating Waypoints----")
        wp = []

        wp.append([-0.5,0.0,-.75, 0.0 , 0.0,float("nan")])
        
        self.get_logger().info("----Waypoint generation completed!----")
        return wp

    def vehicle_odometry_callback(self, vehicle_odometry):
        self.vehicle_odometry = vehicle_odometry

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def arm(self):
        """send an arm command to the vehicle"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1 = 1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z:float):
        msg = TrajectorySetpoint()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.yaw = 0.0 # 0 degree
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:

        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter +=1        

        # Thres_err based waypoint follower
        if self.err > self.thres_err and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(self.waypoints[self.wp_num][0], self.waypoints[self.wp_num][1], self.waypoints[self.wp_num][2], self.waypoints[self.wp_num][3], self.waypoints[self.wp_num][4], self.waypoints[self.wp_num][5] )

        elif self.err <= self.thres_err and self.wp_num < len(self.waypoints) and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(self.waypoints[self.wp_num][0], self.waypoints[self.wp_num][1], self.waypoints[self.wp_num][2], self.waypoints[self.wp_num][3], self.waypoints[self.wp_num][4], self.waypoints[self.wp_num][5])
            self.wp_num +=1

        # Thres_err based waypoint follower: Landing setpoint after reaching the last waypoint
        elif self.err <= self.thres_err and self.wp_num >= len(self.waypoints):
            self.publish_position_setpoint(0.0, 0.0, 0.0, float("nan"),float("nan"),float("nan"))     
            exit(0)

        # Compute error using des_setpoint - curr_setpoint
        if self.wp_num < len(self.waypoints):
            self.err = math.sqrt((self.vehicle_odometry.x - self.waypoints[self.wp_num][0])**2 + (self.vehicle_odometry.y - self.waypoints[self.wp_num][1])**2 +(self.vehicle_odometry.z - self.waypoints[self.wp_num][2])**2)
    

        self.get_logger().info(f"Error: {self.err}")


def main(args = None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = PulleyPull()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
    