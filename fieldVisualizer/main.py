import sys
import pygame
import geometry
import math
from geometry import *
from pygame.locals import *
import net
import fieldData

DEBUG = True

RED = (255, 0, 0)
GREEN = (0, 255, 128)
BLUE = (0, 0, 255)
MAGENTA = (255, 0, 255)
ORANGE = (255, 95, 31)
GRAY = (169, 169, 169)

pygame.init()
pygame.font.init()

from text import Text

SCREEN_WIDTH = 1134
SCREEN_HEIGHT = 580

screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT))

font = pygame.font.SysFont("Consolas", 18)

pygame.display.set_caption("2023 Field")

field = Field(Vector2(46, 36), Vector2(1088, 544), Vector2(16.5418, 8.0137), "assets/2023-field.png")

charging_station_coordinates = [
    [(2.9, 1.5), (2.9, 4.0), (4.9, 4.0), (4.9, 1.5)],
    [(11.65, 1.5), (11.65, 4.0), (13.65, 4.0), (13.65, 1.5)],
]

grid_coordinates = [
    [(0, 5.5), (1.4, 5.5), (1.4, 0), (0, 0)],
    [(15, 5.5), (16.4, 5.5), (16.4, 0), (15, 0)]
]

charging_station_obstacles = [PolygonalObstacle([Vector2(point[0], point[1], VectorType.FIELD) for point in obstacle_points]) for obstacle_points in charging_station_coordinates]
grid_obstacles = [PolygonalObstacle([Vector2(point[0], point[1], VectorType.FIELD) for point in obstacle_points]) for obstacle_points in grid_coordinates]


obstacles = [
    *charging_station_obstacles,
    *grid_obstacles
]

obstacle_buffers = [
    *[obstacle.buffer for obstacle in charging_station_obstacles],
    *[obstacle.buffer for obstacle in grid_obstacles]
]

last_clicked_mouse = None

end_position = Vector2(14.4, 2.7)

def estimate_angles(points):
    if len(points) < 2: raise ValueError()
    elif len(points) == 2:
        theta = math.atan2(points[1].y - points[0].y, points[1].x - points[0].x)
        return (Rotation2d(theta), Rotation2d(theta))
    else:
        theta1 = math.atan2(points[1].y - points[0].y, points[1].x - points[0].x)
        theta2 = math.atan2(points[-1].y - points[-2].y, points[-1].x - points[-2].x)
        
        return (Rotation2d(theta1), Rotation2d(theta2))

vis_graph_entry = net.getTable("trajectory").getEntry("visibilityGraph")
placement_serialized_entry = net.getTable("Preferences").getEntry("PiecePlacement/data")

while True:
    mouse_pos = Vector2(pygame.mouse.get_pos())

    net.getTable("trajectory").putBoolean("debug", DEBUG)

    for event in pygame.event.get():              
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.MOUSEBUTTONUP:
            last_clicked_mouse = Vector2(mouse_pos)

    # Retrieve robot poses from networktables

    robot_odometry_pose = net.getTable("SmartDashboard/Field").getNumberArray("Robot", [0, 0, 0])
    robot_limelight_pose = net.getTable("limelight").getNumberArray("botpose", [0, 0, 0])
    if len(robot_limelight_pose) == 0: robot_limelight_pose = [0, 0, 0]

    robot_current_trajectory = net.getTable("trajectory").getNumberArray("currentTrajectory", [0, 0, 0, 0])
    current_trajectory_line = Line([Vector2(robot_current_trajectory[i], robot_current_trajectory[i + 1], VectorType.FIELD) for i in range(0, len(robot_current_trajectory) - 1, 2)])

    robot_position = {
        "limelight": {
            "pos": Vector2(robot_limelight_pose[:2], None, VectorType.LIMELIGHT),
            "rot": 0 # TODO
        },
        "odometry": {
            "pos": Vector2(robot_odometry_pose[:2], None, VectorType.FIELD),
            "rot": robot_odometry_pose[2]
        }
    }

    field.draw(screen)

    byte_data = vis_graph_entry.getRaw(None)
    graph = net.parse_visibility_graph(byte_data)
    # draw the visibility graph
    for key in graph:
        start_point = key
        for p in graph[key]:
            LineSegment(start_point, p).draw(screen, GRAY, field, width=1)

    # display placement positions
    placement_positions = net.parse_placement_points(placement_serialized_entry.getString(""))
    for v in placement_positions:
        v.draw(screen, GREEN, field)

    for obstacle in fieldData.avoidance_obstacles:
        obstacle.draw(screen, ORANGE, field, width=2)

    robot_accepted_positition = robot_position["odometry"]["pos"].get_field(field)
    
    current_trajectory_line.draw(screen, MAGENTA, field, width=4)

    robot_polygon = geometry.generateRobotPolygon(field.to_display_coords(robot_accepted_positition), robot_position["odometry"]["rot"] / 180 * pi)
    robot_accepted_positition.draw(screen, GREEN, field)
    robot_polygon.draw(screen, ORANGE, field, width=4)

    robot_coords_text = Text("Robot: ({:.2f}, {:.2f})".format(robot_accepted_positition.x, robot_accepted_positition.y), 0, 0)
    mouse_coords_text = Text("Mouse: ({:.2f}, {:.2f})".format(mouse_pos.get_field(field).x, mouse_pos.get_field(field).y), SCREEN_WIDTH - 230, 0)

    if last_clicked_mouse:
        last_mouse_coords_text = Text("({:.2f}, {:.2f})".format(last_clicked_mouse.get_field(field)[0], last_clicked_mouse.get_field(field)[1]), 0, 0)
        last_mouse_coords_text.x = last_clicked_mouse.x
        last_mouse_coords_text.y = last_clicked_mouse.y
        last_mouse_coords_text.draw(screen)
        pygame.draw.circle(screen, GREEN, [*last_clicked_mouse], 2)
        net.getTable("trajectory").putNumberArray("visualizerPoint", [last_clicked_mouse.get_field(field)[0], last_clicked_mouse.get_field(field)[1]])

    robot_coords_text.draw(screen)
    mouse_coords_text.draw(screen)

    pygame.display.update()