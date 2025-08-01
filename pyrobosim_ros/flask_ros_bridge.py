from flask import Flask, request, jsonify
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import threading

app = Flask(__name__)

# ROS2 Node Wrapper
class FlaskROSBridge(Node):
    def __init__(self, robot_name="robot"):
        super().__init__('flask_ros_bridge')
        self.robot_name = robot_name
        self.pose_pub = self.create_publisher(PoseStamped, f"{robot_name}/cmd_pose", 10)
        self.vel_pub = self.create_publisher(Twist, f"{robot_name}/cmd_vel", 10)

    def publish_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        import math
        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)

    def publish_vel(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.vel_pub.publish(msg)

# Start ROS2 in background thread
rclpy.init()
ros_bridge = FlaskROSBridge()
ros_thread = threading.Thread(target=rclpy.spin, args=(ros_bridge,), daemon=True)
ros_thread.start()

@app.route("/cmd_pose", methods=["POST"])
def cmd_pose():
    data = request.json
    x = data.get("x")
    y = data.get("y")
    yaw = data.get("yaw", 0.0)
    ros_bridge.publish_pose(x, y, yaw)
    return jsonify({"status": "pose command sent"})

@app.route("/cmd_vel", methods=["POST"])
def cmd_vel():
    data = request.json
    vx = data.get("vx", 0.0)
    vy = data.get("vy", 0.0)
    wz = data.get("wz", 0.0)
    ros_bridge.publish_vel(vx, vy, wz)
    return jsonify({"status": "velocity command sent"})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
