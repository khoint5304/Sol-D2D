import sys
import re
import csv
import os

def extract_data(output):
    thoi_gian = "N/A"
    truck_routes = "N/A"
    drone_routes = "N/A"

    # Extract completion time
    thoi_gian_match = re.search(r"Minimum completion time: (\S+)", output)
    if thoi_gian_match:
        thoi_gian = thoi_gian_match.group(1)

    # Extract truck routes
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
    output = sys.stdin.read().strip()  # Remove any extra newline characters from input
    dronetype = os.environ.get("DRONETYPE")
    data_file = os.environ.get("DATA_FILE")

    if not dronetype or not data_file:
        print("Error: DRONETYPE or DATA_FILE environment variable not set.")
        sys.exit(1)

    # Extract data from output
    thoi_gian, truck_routes, drone_routes = extract_data(output)
    route = f"Truck: {truck_routes} || Drone: {drone_routes}"

    # Get the file name from the data file path
    file_name = os.path.basename(data_file).strip()

    # Ensure data is valid before appending to results.csv
    if thoi_gian == "N/A" or truck_routes == "N/A" or drone_routes == "N/A" or not file_name:
        print("Error: Invalid data extracted. Skipping writing to CSV.")
        sys.exit(1)

    csv_file = "results.csv"
    file_exists = os.path.exists(csv_file)

    # Write to results.csv
    with open(csv_file, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        if not file_exists:
            writer.writerow(["Dronetype", "Thoi Gian", "Route", "Problem"])  # Write header if file does not exist
        writer.writerow([dronetype.strip(), thoi_gian.strip(), route.strip(), file_name])

    print(f"Data appended to {csv_file}")
