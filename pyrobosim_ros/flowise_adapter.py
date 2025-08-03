import requests
import os, time
import json
from dotenv import load_dotenv

ROS_URL = "http://localhost:5000/lidar"
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
    payload = {"question": json.dumps(lidar_data)}
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

def main():
    while True:
        data = fetch_lidar()
        if data:
            print("Lidar data:", data)
            flowise_response = send_to_flowise(data)
            if flowise_response:
                print("Flowise Modellantwort:", flowise_response)
        time.sleep(INTERVAL)

if __name__ == "__main__":
    main()
