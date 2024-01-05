import pygame
from math import sin, cos, pi
import pyvisgraph as vg
from enum import Enum

def sgn(n):
    if n < 0:
        return -1
    else: return 1

def rot_origin(vec2, theta):
    return Vector2(vec2.x * cos(theta) - vec2.y * sin(theta), vec2.x * sin(theta) + vec2.y * cos(theta))

def cartesian_to_window(vec2, w, h):
    return (vec2.x, h - vec2.y)

class VectorType(Enum):
    FIELD = 1
    WINDOW = 2
    LIMELIGHT = 3

class Drawable:
    def draw(self, screen, color = None, *args, **kwargs):
        pass

class Vector2(Drawable):
    def __init__(self, x, y=None, type=VectorType.WINDOW):
        if y == None:
            self.x = x[0]
            self.y = x[1]
        else:
            self.x = x
            self.y = y

        self.type = type

    def rotate(self, theta):
        '''
        Rotate the vector by theta degrees around the origin
        '''

        x = self.x * cos(theta) - self.y * sin(theta)
        y = self.x * sin(theta) + self.y * cos(theta)

        self.x = x
        self.y = y

        return self

    def get_window(self, field):
        if self.type == VectorType.FIELD:
            return field.to_display_coords(self)
        elif self.type == VectorType.LIMELIGHT:
            return field.limelight_to_field(self).get_window()
        else:
            return self
        
    def get_field(self, field):
        if self.type == VectorType.FIELD:
            return self
        elif self.type == VectorType.LIMELIGHT:
            return field.limelight_to_field(self).get_field()
        else:
            return field.to_field_coords(self)

    def draw(self, screen, color, field, *args, **kwargs):
        return pygame.draw.circle(screen, color, (self.get_window(field).x, self.get_window(field).y), 2, *args, **kwargs)

    def __getitem__(self, key):
        if key > 1: raise IndexError()
        if key == 0: return self.x
        else: return self.y

class LineSegment(Drawable):
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def draw(self, screen, color, field, *args, **kwargs):
        pygame.draw.line(screen, color, [*self.start.get_window(field)], [*self.end.get_window(field)], *args, **kwargs)

class Line(Drawable):
    def __init__(self, points):
        self.points = points

    def draw(self, screen, color, field, *args, **kwargs):
        return pygame.draw.lines(screen, color, False, [[*point.get_window(field)] for point in self.points], *args, **kwargs)

    @staticmethod
    def from_trajectory(trajectory):
        line = Line([])
        for state in trajectory.states():
            pose = state.pose
            line.points.append(Vector2(pose.X(), pose.Y(), VectorType.FIELD))

        return line

class Polygon(Drawable):
    def __init__(self, points):
        self.points = points

    def draw(self, screen, color, field, *args, **kwargs):
        pygame.draw.polygon(screen, color, [[*point.get_window(field)] for point in self.points], *args, **kwargs)

class PolygonalObstacle(Polygon):
    def __init__(self, points, buffer_x = 0.4,  buffer_y = 0.4):
        Polygon.__init__(self, points)
        x_avg = sum([p.x for p in points]) / len(points)
        y_avg = sum([p.y for p in points]) / len(points)

        self.buffer = Polygon([Vector2(buffer_x * sgn(p.x - x_avg) + p.x, buffer_y * sgn(p.y - y_avg) + p.y, VectorType.FIELD) for p in points])

    def draw(self, screen, color, field, *args, **kwargs):
        pygame.draw.polygon(screen, color, [[*point.get_window(field)] for point in self.points], *args, **kwargs)
        self.buffer.draw(screen, (255, 255, 0), field, width=1)

class Field(Drawable):
    def __init__(self, topLeft, bottomRight, dimension, field_image):
        self.topLeft = topLeft
        self.bottomRight = bottomRight
        self.dimension = dimension
        self.pixelsPerMeter = 0.5 * (((bottomRight.x - topLeft.x) / dimension.x) + ((bottomRight.y - topLeft.y) / dimension.y))
        
        if type(field_image) == str: self.field_image = pygame.image.load(field_image).convert()
        else: self.field_image = field_image

    def limelight_to_field(self, limelight_coords):
        return Vector2(self.topLeft.x + self.dimension.x / 2 + limelight_coords.x, self.topLeft.y + self.dimension.y / 2 + limelight_coords.y, VectorType.FIELD)
    
    def to_display_coords(self, coords):
        return Vector2(self.topLeft.x + coords.x * self.pixelsPerMeter, self.topLeft.y + (self.dimension.y - coords.y) * self.pixelsPerMeter)

    def to_field_coords(self, coords):
        return Vector2((coords.x - self.topLeft.x) / self.pixelsPerMeter, self.dimension.y - (coords.y - self.topLeft.y) / self.pixelsPerMeter)

    def draw(self, screen, color=None, *args, **kwargs):
        return screen.blit(self.field_image, (0, 0))

def build_shortest_path(start, end, obstacles=[]):
    polygons = [[vg.Point(point.x, point.y) for point in obstacle.points] for obstacle in obstacles]
    g = vg.VisGraph()
    g.build(polygons)

    shortest_path = g.shortest_path(vg.Point(start.x, start.y), vg.Point(end.x, end.y))
    return Line([Vector2(p.x, p.y, VectorType.FIELD) for p in shortest_path])

def generateRobotPolygon(pos, rot):
    width = 30
    height = 25

    coords = [
        Vector2(width/2, 0).rotate(2 * pi - rot),
        Vector2(-width/2, height/2).rotate(2 * pi - rot),
        Vector2(-width/2, -height/2).rotate(2 * pi - rot)
    ]

    return Polygon([
        Vector2(pos.x + coords[0].x, pos.y + coords[0].y, VectorType.WINDOW),
        Vector2(pos.x + coords[1].x, pos.y + coords[1].y, VectorType.WINDOW),
        Vector2(pos.x + coords[2].x, pos.y + coords[2].y, VectorType.WINDOW)
    ])