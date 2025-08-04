import requests
import os, time
import json
from dotenv import load_dotenv

ROS_URL = "http://localhost:5000/lidar"
CMD_POSE_URL = "http://localhost:5000/cmd_pose"
CMD_VEL_URL = "http://localhost:5000/cmd_vel"
INTERVAL = 0.5  # 500ms, aber wir warten immer auf die Modellantwort

# Load configuration from .env file
load_dotenv()
FLOW_ID = os.getenv('FLOWID')
FLOWISE_TOKEN = os.getenv('FLOWISE_TOKEN')
FLOWISE_URL = f"http://localhost:3000/api/v1/prediction/{FLOW_ID}"

def fetch_lidar():
    try:
        response = requests.get(ROS_URL, timeout=2)
        response.raise_for_status()
        return response.json()
    except Exception as e:
        print(f"Error fetching lidar data: {e}")
        return None

def send_to_flowise(lidar_data):
    # Nur relevante Felder extrahieren
    reduced_data = {
        "range_min": lidar_data.get("range_min"),
        "range_max": lidar_data.get("range_max"),
        "ranges": lidar_data.get("ranges", [])
    }
    payload = {"question": reduced_data}  # ACHTUNG: kein json.dumps mehr, sondern echtes JSON-Objekt
    #print("Flowise Payload:", json.dumps(payload, indent=2))
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {FLOWISE_TOKEN}"
    }
    try:
        response = requests.post(FLOWISE_URL, json=payload, headers=headers, timeout=10)
        print("Flowise Status:", response.status_code)
        print("Flowise Antwort:", response.text)
        response.raise_for_status()
        return response.json()
    except Exception as e:
        print(f"Error sending to Flowise: {e}")
        return None

def send_cmd_pose(cmd_pose):
    try:
        response = requests.post(CMD_POSE_URL, json=cmd_pose, timeout=2)
        print("cmd_pose Status:", response.status_code)
        print("cmd_pose Antwort:", response.text)
    except Exception as e:
        print(f"Error sending cmd_pose: {e}")

def send_cmd_vel(cmd_vel):
    try:
        response = requests.post(CMD_VEL_URL, json=cmd_vel, timeout=2)
        print("cmd_vel Status:", response.status_code)
        print("cmd_vel Antwort:", response.text)
    except Exception as e:
        print(f"Error sending cmd_vel: {e}")

def parse_and_send_commands(flowise_response):
    try:
        text = flowise_response.get('text', '')
        # Nur weitermachen, wenn ein JSON-Block enthalten ist
        if '```json' in text or '```' in text:
            if '```json' in text:
                text = text.split('```json')[1].split('```')[0]
            else:
                text = text.split('```')[1].split('```')[0]
            cmd_data = json.loads(text)
            if 'cmd_pos' in cmd_data:
                send_cmd_pose(cmd_data['cmd_pos'])
            if 'cmd_vel' in cmd_data:
                send_cmd_vel(cmd_data['cmd_vel'])
        else:
            print("No JSON command in Flowise response. Text:", text)
    except Exception as e:
        print(f"Error parsing/sending commands: {e}")

def main():
    while True:
        data = fetch_lidar()
        if data:
            #print("Lidar data:", data)
            flowise_response = send_to_flowise(data)
            if flowise_response:
                print("Flowise Modellantwort:", flowise_response)
                parse_and_send_commands(flowise_response)
        time.sleep(INTERVAL)

if __name__ == "__main__":
    main()
