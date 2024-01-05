from networktables import NetworkTables
import struct
from geometry import Vector2, VectorType

initializeState = False

if not initializeState:
    print("Initializing NetworkTables")
    NetworkTables.initialize(server='localhost')
    initializeState = True

def getTable(key):
    return NetworkTables.getTable(key)

def parse_placement_points(str):
    byte_arr = bytes(str, "utf-16-le")

    arr = []
    for i in struct.iter_unpack(">d", byte_arr):
        arr.append(i[0])
    
    points = []
    for i in range(0, len(arr), 2):
        points.append(Vector2(arr[i], arr[i + 1], VectorType.FIELD))

    return points

def parse_visibility_graph(byte_data):
    '''
    Parses a visibility graph from the data sent over networktables
    '''
    if not byte_data: return {}

    arr = []
    for i in struct.iter_unpack(">f", byte_data):
        arr.append(i[0])

    points = []
    graph = {}
    index = 0
    entries = int(arr[index])
    index += 1
    for _ in range(entries):
        points.append(Vector2(arr[index], arr[index + 1], VectorType.FIELD))
        index += 2

    seg_idx = 0
    while index < len(arr):
        seg_len = int(arr[index])
        index += 1
        p = points[seg_idx]
        seg_idx += 1
        graph[p] = []
        for _ in range(seg_len):
            graph[p].append(points[int(arr[index])])
            index += 1

    return graph