import sys
import re
import csv
import os

def extract_data(output):
    thoi_gian = "N/A"
    truck_routes = "N/A"
    drone_routes = "N/A"

    thoi_gian_match = re.search(r"Minimum completion time: (\S+)", output)
    if thoi_gian_match:
        thoi_gian = thoi_gian_match.group(1)

    truck_routes_match = re.search(r"Truck routes:\n(.*?)\nDrone routes:", output, re.DOTALL)
    if truck_routes_match:
        truck_routes_raw = truck_routes_match.group(1).strip()
        truck_routes = "; ".join(line.strip() for line in truck_routes_raw.splitlines() if line.strip())

    # Extract drone routes
    drone_routes_match = re.search(r"Drone routes:\n(.*?)(?=$|\n)", output, re.DOTALL)
    if drone_routes_match:
        drone_routes_raw = drone_routes_match.group(1).strip()
        drone_routes = "; ".join(line.strip() for line in drone_routes_raw.splitlines() if line.strip())

    return thoi_gian, truck_routes, drone_routes

if __name__ == "__main__":
    output = sys.stdin.read()
    dronetype = os.environ.get("DRONETYPE")
    data_file = os.environ.get("DATA_FILE")

    if dronetype is None or data_file is None:
        print("Error: DRONETYPE or DATA_FILE environment variable not set.")
        sys.exit(1)

    thoi_gian, truck_routes, drone_routes = extract_data(output)
    route = f"Truck: {truck_routes} || Drone: {drone_routes}"

    # Lấy tên file từ đường dẫn
    file_name = os.path.basename(data_file)

    csv_file = "results.csv"
    file_exists = os.path.exists(csv_file)

    with open(csv_file, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        if not file_exists:
            writer.writerow(["Dronetype", "Thoi Gian", "Route", "Problem"])
        writer.writerow([dronetype, thoi_gian, route, file_name])

    print(f"Data appended to {csv_file}")