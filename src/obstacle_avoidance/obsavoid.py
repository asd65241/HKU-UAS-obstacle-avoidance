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


class Mission:
    def __init__(self, data):
        self.data = data

        # Color
        self.RED = "#eb4d4b"
        self.GREEN = "#22a6b3"
        self.BLUE = "#130f40"
        self.ORANGE = "#f0932b"
        self.YELLOW = "#f9ca24"

        # Define coordinates of where we want to center our map
        self.boulder_coords = [22.274097, 114.122616]

        # Create the map
        self.my_map = folium.Map(location=self.boulder_coords, zoom_start=20)

    def obs_avoid(self, RESOLUTION=20, CIRCLE_OFFSET=10):
        self.RESOLUTION = RESOLUTION
        self.CIRCLE_OFFSET = CIRCLE_OFFSET

        start_time = time.perf_counter()

        # number of process
        num_process = 0
        num_done = 0
        num_submited = 0

        # Input data
        home_point = self.data['mission']['plannedHomePosition']
        circleArr = self.data["geoFence"]["circles"]
        boundaryArr = self.data["geoFence"]["polygons"][0]['polygon']
        waypoint = []

        waypoint.append(home_point[0:2])

        # Create waypoint
        for i in self.data["mission"]["items"]:
            if 'TransectStyleComplexItem' in i:
                waypoint.append('search_grid')
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
            zoneLetter = from_latlon(temp_x, temp_y)[3]
            temp = from_latlon(temp_x, temp_y)[0:2]
            # print(f"ZoneNum: {zoneNum} {zoneLetter}")
            boundaryUTM.append(temp)

        for i in waypoint:
            if i != 'search_grid':
                xy = i
                temp_x, temp_y = xy
                zoneNum = from_latlon(temp_x, temp_y)[2]
                temp = from_latlon(temp_x, temp_y)[0:2]
                waypointUTM.append(temp)
            else:
                waypointUTM.append('search_grid')

        print(waypointUTM)

        # Create Map Elements
        bountdaryPath = path.Path(boundaryUTM)
        circlePath = []
        for circle in circleUTM:
            circlePath.append(path.Path.circle(center=circle["UTM"], radius=(
                int(circle["radius"])+CIRCLE_OFFSET), readonly=True))

        linePath = []
        for i in range(len(waypointUTM)-1):
            waypath = waypointUTM[i:i+2]
            if 'search_grid' in waypoint:
                continue
            codes = [path.Path.MOVETO, path.Path.LINETO]
            linePath.append(path.Path(waypath, codes))




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
                color=self.ORANGE
            ).add_to(self.my_map)

        # Create Polygon
        folium.Polygon(
            locations=boundaryArr,
            fill=False,
            tooltip="GeoFence",
            color=self.RED).add_to(self.my_map)

        # Create Waypath
        # for i in range(len(waypoint)-1):
        #     if not hitCircle(linePath[i]):
        #         folium.PolyLine(
        #             locations=waypoint[i:i+2],
        #             fill=False,
        #             tooltip=f"Waypath {i}",
        #             color=GREEN,
        #             opacity=(0.3),
        #         ).add_to(my_map)



        # Create autopilot route
        route = []

        for i in range(len(waypoint)-1):
            if 'search_grid' in waypointUTM[i] or 'search_grid' in waypointUTM[i+1]:
                route.append('search_grid')
            else:
                start_x, start_y = waypointUTM[i]
                end_x, end_y = waypointUTM[i+1]

                start = tuple((int(start_x), int(start_y)))
                end = tuple((int(end_x), int(end_y)))
                route.append({"start": start, "end": end})

        path_final = []
        with concurrent.futures.ProcessPoolExecutor() as executor:
            
            num_process = len(route) - route.count('search_grid')
            print(f"Number of process: {num_process}")
            process_pool = []
            search_grid_index = []
            for i in route:
                if i != 'search_grid':
                    process = executor.submit(algorithm, i["start"], i["end"], RESOLUTION, circlePath, bountdaryPath, zoneNum, zoneLetter)
                    process_pool.append(process)
                    num_submited += 1
                    print(f"Process Submitted: {num_submited}")
                else:
                    search_grid_index.append(num_submited)

            # Add path
            for path_ in process_pool:
                if num_done in search_grid_index:
                    path_final.append('search_grid')
                path_final.extend(path_.result())
                #print(path_.result())
                num_done += 1
                print(f"Process Finished: {num_done}/{num_process}")

        # Remove duplicate
        path_final = list(dict.fromkeys(path_final))

        # print(path_final)

        # Add Autopilot WayPath to Map
        index = 0
        num_of_element = len(path_final) - path_final.count('search_grid')
        for i in range(num_of_element-1):
            if path_final[i] == 'search_grid' or path_final[i+1] == 'search_grid':
                continue
            else:
                path_ = []
                path_.append(path_final[i])
                path_.append(path_final[i+1])
                index += 1
                msg = f"index: {i}"
                folium.PolyLine(
                    locations=path_,
                    fill=False,
                    tooltip=msg,
                    color=self.YELLOW
                ).add_to(self.my_map)

        # Add Waypoint to map
        num = 0
        for point in waypoint:
            if point != 'search_grid':
                num += 1
                lat, lot = point
                point_utm = from_latlon(lat, lot, zoneNum, zoneLetter)[0:2]
                msg = str(inCircle(*point_utm, circlePath))
                folium.CircleMarker(
                    location=point,
                    fill=True,
                    popup=msg,
                    radius=5,
                    fill_opacity=1,
                    color=self.BLUE
                ).add_to(self.my_map)

        # Display the map
        self.my_map

        # Save the map
        self.my_map.save("map.html")

        # Make flight plan for QGC

        items = self.data['mission']['items']

        take_off = None
        landing = None
        search_grid = []
        altitude = 0

        waypoint_items = []

        for id, mission_item in enumerate(items):
            if 'command' in mission_item:
                if mission_item['command'] == 22:
                    take_off = mission_item
                    altitude = mission_item['Altitude']
                elif mission_item['command'] == 20:
                    landing = mission_item
            elif 'TransectStyleComplexItem' in mission_item:
                search_grid.append(mission_item)

        waypoint_items.append(take_off)

        for xy in path_final:
            if xy == 'search_grid':
                waypoint_items.append(search_grid.pop(0))
            else:
                lat, lot = xy
                template = {'AMSLAltAboveTerrain': altitude, 'Altitude': altitude, 'AltitudeMode': 1, 'autoContinue': True, 'command': 16,
                            'doJumpId': 2, 'frame': 3, 'params': [0, 0, 0, None, lat, lot, altitude], 'type': 'SimpleItem'}

                waypoint_items.append(template)

        waypoint_items.append(landing)

        self.data['mission']['items'] = waypoint_items

        # Calculate the execution time
        finish_time = time.perf_counter()
        print(f'Number of waypoint: {len(path_final)}')
        print(f'Finished in {round(finish_time-start_time, 2)} second(s)')

        return self.data


    # Export flight plan for QGC

    def export(self):
        current_time = time.strftime("Flight_%Y%m%d_%H-%M-%S", time.localtime())
        with open(f'{current_time}.plan', 'w') as outfile:
            json.dump(self.data, outfile)
        
        print(f'Exported Mission File: {current_time}.plan')

    
    # def get_waypoints(self):
    #     self.data





#functions for A* algo
def hitCircle(path, circlePath):
    for i in circlePath:
        if path.contains_path(i):
            return True
    return False


def inCircle(utmx, utmy, circlePath):
    for i in circlePath:
        if i.contains_points([(utmx, utmy)]):
            return True
    return False


def inBound(utmx, utmy, bountdaryPath):
    if bountdaryPath.contains_points([(utmx, utmy)]):
        return True
    else:
        return False


def hitPoint(utmx, utmy):
    pass


def slope(pt1, pt2):
    pt1_x, pt1_y = pt1
    pt2_x, pt2_y = pt2

    y = pt2_y-pt1_y
    x = pt2_x-pt1_x

    if (x == 0):
        x = 0.00001

    m = y/x

    return m

# A star

# Euclidean Distance


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2))


def reconstruct_path(came_from, current, res, zoneNum, zoneLetter):
    coordinate = []

    coordinate.append(current)

    for point in came_from.keys():
        if h(current, point) < res:
            current = point

    while current in came_from:
        current = came_from[current]
        output = current
        coordinate.append(output)

    coordinate.reverse()

    output = []
    top = None
    num_of_element = len(coordinate)-2

    output.append(to_latlon(*coordinate[0], zoneNum, zoneLetter))
    for i in range(num_of_element):
        start = coordinate[i]
        check = coordinate[i+1]
        end = coordinate[i+2]

        # print(abs(slope(start, check) - slope(check, end)))

        if abs(slope(start, check) - slope(check, end)) <= 0.05:
            pass
        else:
            output.append(to_latlon(*check, zoneNum, zoneLetter))

    output.append(to_latlon(*coordinate[-1], zoneNum, zoneLetter))

    return output


def algorithm(start, end, RESOLUTION, circlePath, bountdaryPath, zoneNum, zoneLetter):
    print("Start Algorithm")
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {start: float("inf")}
    g_score[start] = 0
    f_score = {start: float("inf")}
    f_score[start] = h(start, end)

    res = RESOLUTION

    # print(f"start: {start}")
    # print(f"end: {end}")

    open_set_hash = {start}

    while not open_set.empty():

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if h(current, end) < res:
            output = reconstruct_path(came_from, end, res, zoneNum, zoneLetter)

            return output

        current_neighbors = []

        # Get x and y
        temp_x, temp_y = current
        # Check inbound and not in circle

        diagonal = math.sqrt(2 * pow(res, 2))
        radius = res

        for i in range(360):
            theta = i * 2 * math.pi / 360
            new_x = temp_x + radius * math.cos(theta)
            new_y = temp_y + radius * math.sin(theta)
            if inBound(new_x, new_y, bountdaryPath) and not inCircle(new_x, new_y, circlePath):
                current_neighbors.append((new_x, new_y))

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




