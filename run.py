import json
import math
import folium
from folium.map import Tooltip
from matplotlib import path
from queue import PriorityQueue
from utm.conversion import from_latlon, to_latlon
import concurrent.futures
import time
import numpy as np

start_time = time.perf_counter()

# Color
RED = "#eb4d4b"
GREEN = "#22a6b3"
BLUE = "#130f40"
ORANGE = "#f0932b"
YELLOW = "#f9ca24"

f = open("mission.plan")

data = json.load(f)

f.close()

# Define coordinates of where we want to center our map
boulder_coords = [38.14479831252449, -76.42773638527646]

# Create the map
my_map = folium.Map(location=boulder_coords, zoom_start=15)

# number of process
num_process = 0
num_done = 0
num_submited = 0

# WSG64
circleArr = data["geoFence"]["circles"]
boundaryArr = data["geoFence"]["polygons"][0]['polygon']
waypoint = []

# Create waypoint
for i in data["mission"]["items"]:
    if ('params' in i):
        case_1 = [None, None]
        case_2 = [0, 0]
        check = i['params'][4:6]
        if (check != case_1 and check != case_2):
            point = i['params'][4:6]
            waypoint.append(point)

start = waypoint[0]
end = waypoint[len(waypoint)-1]

# UTM
zoneNum = 0
zoneLetter = ''
circleUTM = []
boundaryUTM = []
waypointUTM = []

for i in circleArr:
    circle = i['circle']
    center = circle['center']
    radius = circle['radius']
    temp_x, temp_y = center
    zoneNum = from_latlon(temp_x, temp_y)[2]
    zoneLetter = from_latlon(temp_x, temp_y)[3]
    temp = from_latlon(temp_x, temp_y)[0:2]
    circleUTM.append({"UTM": temp, "radius": radius})

for i in boundaryArr:
    polygon = i
    temp_x, temp_y = polygon
    zoneNum = from_latlon(temp_x, temp_y)[2]
    temp = from_latlon(temp_x, temp_y)[0:2]
    boundaryUTM.append(temp)

for i in waypoint:
    xy = i
    temp_x, temp_y = xy
    zoneNum = from_latlon(temp_x, temp_y)[2]
    temp = from_latlon(temp_x, temp_y)[0:2]
    waypointUTM.append(temp)

# Create Map Elements
bountdaryPath = path.Path(boundaryUTM)
circlePath = []
for circle in circleUTM:
    circlePath.append(path.Path.circle(
        center=circle["UTM"], radius=circle["radius"], readonly=False))

linePath = []
for i in range(len(waypointUTM)-1):
    waypath = waypointUTM[i:i+2]
    codes = [path.Path.MOVETO,path.Path.LINETO]
    linePath.append(path.Path(waypath,codes))

def hitCircle(path):
    for i in circlePath:
        if path.contains_path(i):
            return True
    return False

def inCircle(utmx, utmy):
    for i in circlePath:
        if i.contains_points([(utmx, utmy)]):
            return True
    return False


def inBound(utmx, utmy):
    if bountdaryPath.contains_points([(utmx, utmy)]):
        return True
    else:
        return False


# A star

# Euclidean Distance
def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2))


def reconstruct_path(came_from, current, res):
    coordinate = []

    for point in came_from.keys():
        if h(current, point) < res:
            coordinate.append(to_latlon(*current, zoneNum, zoneLetter))
            current = point

    while current in came_from:
        current = came_from[current]
        output = to_latlon(*current, zoneNum, zoneLetter)
        coordinate.append(output)

    return coordinate


def algorithm(start, end):
    print("Start Algorithm")
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {start: float("inf")}
    g_score[start] = 0
    f_score = {start: float("inf")}
    f_score[start] = h(start, end)

    res = 10

    # print(f"start: {start}")
    # print(f"end: {end}")

    open_set_hash = {start}

    while not open_set.empty():

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if h(current,end) < res:
            output = reconstruct_path(came_from, end, res)
            return output

        current_neighbors = []

        # Get x and y
        temp_x, temp_y = current
        # Check inbound and not in circle

        diagonal = math.sqrt(2* pow(res,2))

        # Check down
        if inBound(temp_x, temp_y + res) and not inCircle(temp_x, temp_y + res):
            current_neighbors.append((temp_x, temp_y + res))

        # Check down left
        if inBound(temp_x - diagonal, temp_y + diagonal) and not inCircle(temp_x - diagonal, temp_y + diagonal):
            current_neighbors.append((temp_x - diagonal, temp_y + diagonal))
        
        # Check down right
        if inBound(temp_x + diagonal, temp_y + diagonal) and not inCircle(temp_x + diagonal, temp_y + diagonal):
            current_neighbors.append((temp_x + diagonal, temp_y + diagonal))
        
        # Check up
        if inBound(temp_x, temp_y - res) and not inCircle(temp_x, temp_y - res):
            current_neighbors.append((temp_x, temp_y - res))

        # Check up left
        if inBound(temp_x - diagonal, temp_y - diagonal) and not inCircle(temp_x - diagonal, temp_y - diagonal):
            current_neighbors.append((temp_x - diagonal, temp_y - diagonal))

        # Check up right
        if inBound(temp_x + diagonal, temp_y - diagonal) and not inCircle(temp_x + diagonal, temp_y - diagonal):
            current_neighbors.append((temp_x + diagonal, temp_y - diagonal))

        # Check right
        if inBound(temp_x + res, temp_y) and not inCircle(temp_x + res, temp_y):
            current_neighbors.append((temp_x + res, temp_y))

        # Check left
        if inBound(temp_x - res, temp_y) and not inCircle(temp_x - res, temp_y):
            current_neighbors.append((temp_x - res, temp_y))

        for neighbor in current_neighbors:

            if current not in g_score or current not in f_score:
                g_score[current] = float("inf")
                f_score[current] = float("inf")

            if neighbor not in g_score or neighbor not in f_score:
                g_score[neighbor] = float("inf")
                f_score[neighbor] = float("inf")

            # All edge are 1
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor, end) * 2
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)

    print("exit while loop")
    return None


# Create Circle
for i in circleArr:
    circle = i['circle']
    center = circle['center']
    radius = circle['radius']
    folium.Circle(
        radius=radius,
        location=center,
        fill=True,
        tooltip="Obstacle",
        opacity=0.2,
        color=ORANGE
    ).add_to(my_map)

# Create Polygon
folium.Polygon(
    locations=boundaryArr,
    fill=False,
    tooltip="GeoFence",
    color=RED).add_to(my_map)

# Create Waypath
for i in range(len(waypoint)-1):
    if not hitCircle(linePath[i]):
        folium.PolyLine(
            locations=waypoint[i:i+2],
            fill=False,
            tooltip=f"Waypath {i}",
            color=GREEN
        ).add_to(my_map)

# Create autopilot route
route = []

for i in range(len(waypoint)-1):
    start_x, start_y = waypointUTM[i]
    end_x, end_y = waypointUTM[i+1]

    start = tuple((int(start_x), int(start_y)))
    end = tuple((int(end_x), int(end_y)))
    route.append({"start": start, "end": end})

if __name__ == "__main__":
    path_final = []
    with concurrent.futures.ProcessPoolExecutor() as executor:
        num_process = len(route)
        print(f"Number of process: {len(route)}")
        process_pool = []
        for i in route:
            process = executor.submit(algorithm, i["start"], i["end"])
            process_pool.append(process)
            num_submited += 1
            print(f"Process Submitted: {num_submited}")

        # Add path
        for path in process_pool:
            path_final.append(path.result())
            num_done += 1
            print(f"Process Finished: {num_done}/{num_process}")

    # Create Autopilot WayPath
    index = 0
    for i in path_final:
        index += 1
        msg = f"Path: {index}"
        folium.PolyLine(
            locations=i,
            fill=False,
            tooltip=msg,
            color=YELLOW
        ).add_to(my_map)

    # Create Waypoint
    num = 0
    for point in waypoint:
        num += 1
        lat, lot = point
        point_utm = from_latlon(lat, lot, zoneNum, zoneLetter)[0:2]
        msg = str(inCircle(*point_utm))
        folium.CircleMarker(
            location=point,
            fill=True,
            popup=msg,
            radius=5,
            fill_opacity=1,
            color=BLUE
        ).add_to(my_map)

    # Display the map
    my_map.save("map.html")

    # Calculate the execution time
    finish_time = time.perf_counter()
    print(f'Number of waypoint: {len(np.concatenate(path_final).ravel().tolist())}')
    print(f'Finished in {round(finish_time-start_time, 2)} second(s)')
