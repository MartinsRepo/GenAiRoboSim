import requests
import time

FLASK_URL = "http://localhost:5000/lidar"
INTERVAL = 0.1  # 100ms

def fetch_lidar():
    try:
        response = requests.get(FLASK_URL, timeout=2)
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
