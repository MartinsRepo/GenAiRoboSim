
from flask import Flask, jsonify
import threading
from pyrobosim_ros.ros_interface import WorldROSWrapper

# Annahme: WorldROSWrapper wird im Hauptprozess erzeugt und als Singleton bereitgestellt
try:
    #from pyrobosim_ros.pyrobosim_ros.ros_interface import WorldROSWrapper
    ros_node = WorldROSWrapper._instance  # oder wie die Instanz bereitgestellt wird
except Exception:
    ros_node = None  # Fallback für Tests


app = Flask(__name__)

@app.route("/")
def index():
    return "PyRoboSim Lidar API. Use /lidar/robot for Lidar data."

@app.route("/lidar/robot")
def get_lidar_robot():
    if ros_node is None:
        return jsonify({"error": "ROS node not available"}), 503
    # Liefere alle Lidar-Daten für alle Roboter als Dict
    return jsonify(ros_node.latest_lidar_data)

if __name__ == "__main__":
    threading.Thread(target=app.run, kwargs={"host": "0.0.0.0", "port": 5000}).start()
