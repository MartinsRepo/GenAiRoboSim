import requests
import os, time
from dotenv import load_dotenv

ROS_URL = "http://localhost:5000/lidar"
INTERVAL = 0.1  # 100ms

# Load configuration from .env file
load_dotenv()
FLOW_ID = os.getenv('FLOWID')
FLOWISE_URL = f"http://localhost:3000/v2/agentcanvas/{FLOW_ID}"

def fetch_lidar():
    try:
        response = requests.get(ROS_URL, timeout=2)
        response.raise_for_status()
        return response.json()
    except Exception as e:
        print(f"Error fetching lidar data: {e}")
        return None

def main():
    while True:
        data = fetch_lidar()
        if data:
            print("Lidar data:", data)
        time.sleep(INTERVAL)

if __name__ == "__main__":
    main()
