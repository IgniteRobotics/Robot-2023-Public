import json
import math
from geometry import *

initialized = False

def sgn(n):
    if n < 0:
        return -1
    else: return 1

avoidance_obstacles = []

if not initialized:
    # load avoidance obstacles
    file = open("../src/main/resources/2023-field.json")
    field_details = json.load(file)
    file.close()


    for obstacle in field_details["obstacles"]:
        points = [Vector2(obstacle["points"][i], obstacle["points"][i + 1], VectorType.FIELD) for i in range(0, len(obstacle["points"]), 2)]
        center_point = Vector2(sum([p.x for p in points]) / len(points), sum([p.y for p in points]) / len(points))
        obstacle["buffer"] = 0
        obstacle_polygon = Polygon([Vector2(p.x + obstacle["buffer"] * sgn(p.x - center_point.x), p.y + obstacle["buffer"] * sgn(p.y - center_point.y), VectorType.FIELD) for p in points])
        avoidance_obstacles.append(obstacle_polygon)
    
    initialized = True