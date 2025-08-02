import requests, json
import os
from dotenv import load_dotenv

# Load configuration from .env file
load_dotenv()
FLOW_ID = os.getenv('FLOWID')
FLOWISE_TOKEN = os.getenv('FLOWISE_TOKEN')
FLOWISE_URL = f"http://localhost:3000/api/v1/prediction/{FLOW_ID}"
print(f"Flowise URL: {FLOWISE_URL}")

# dummy Lidar data for testing
lidar_data_payload = {
    "question": json.dumps({
        "stamp": 123456.789,
        "frame_id": "base_link",
        "angle_min": -3.14,
        "angle_max": 3.14,
        "angle_increment": 0.01745,
        "range_min": 0.1,
        "range_max": 10.0,
        "ranges": [1.0, 2.0, 3.0, 4.0, 5.0]
    })
}


try:
    response = requests.post(FLOWISE_URL, json=lidar_data_payload, timeout=5)
    print("Status:", response.status_code)
    print("Text:", response.text)
    response.raise_for_status()
    print("Answer from the model:", response.json())
except Exception as e:
    print(f"Error during transmission to Flowise: {e}")

