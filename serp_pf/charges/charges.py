from mpmath import *
from enum import Enum

class Orientation(Enum):
    Horizontal = 0
    Vertical = 1

# A point in 2D space
class Point:
    def __init__(self, x, y) -> None: self.x , self.y = x, y
    
    # Point A - Point B returns Vector BA
    def __sub__(self, point): return Vector(self.x - point.x, self.y - point.y)

    def __str__(self) -> str: return 'Point(' + str(self.x) + ', ' + str(self.y) + ')'

    # Returns resulting Point of adding Vector vector to Point self
    def add_vector(self, vector): return Point(self.x + vector.x, self.y + vector.y)
    
    # Euclidean distance from self to point
    def distance(self, point): return (self - point).magnitude()

# A vector in 2D space
class Vector:
    def __init__(self, x, y) -> None: self.x , self.y = x, y
    
    def __str__(self) -> str: return 'Vector(' + str(self.x) + ', ' + str(self.y) + ')'

    # Divide vector by numerical value
    def __truediv__(self, value): return Vector(self.x / value, self.y / value)

    # Returns magnitude of the vector
    def magnitude(self): return sqrt ((self.x ** 2) + (self.y ** 2))

# A position in 2D space, point(x, y) and rotation
class Position: 
    def __init__(self, point, rot) -> None: self.point, self.rot = point, rot

# Base charge with intensity and get_force_at_point method
class Charge:
    def __init__(self, intensity) -> None: self.intensity = intensity
    
    # Return force produced in a point by this charge
    def get_force_at_point(self, _): return None

# Charge produced by a point
class PointCharge(Charge):
    def __init__(self, point, intensity) -> None:
        super().__init__(intensity)
        self.point = point

    # override method
    def get_force_at_point(self, point):
        vec = (self.point.x - point.x, self.point.y - point.y)
        vec_r = sqrt((vec[0] ** 2) + (vec[1] ** 2))
        if vec_r == 0: return Vector(0, 0)
        vec_angle = atan2(vec[1], vec[0])

        force_intensity = self.intensity / (vec_r ** 2)

        return Vector(force_intensity * cos(vec_angle), force_intensity * sin(vec_angle))

# Tension produced by a point
class PointTension(Charge):
    def __init__(self, point, intensity) -> None:
        super().__init__(intensity)
        self.point = point

    # override method
    def get_force_at_point(self, point):
        vec = (self.point.x - point.x, self.point.y - point.y)
        vec_r = sqrt((vec[0] ** 2) + (vec[1] ** 2))
        vec_angle = atan2(vec[1], vec[0])

        force_intensity = self.intensity * vec_r

        return Vector(force_intensity * cos(vec_angle), force_intensity * sin(vec_angle))
    
# Charge produced by a line segment(defined by two points)
class LineSegmentCharge(Charge):
    def __init__(self, point1, point2, intensity) -> None:
        super().__init__(intensity)
        if point1.y == point2.y: self.orientation = Orientation.Horizontal
        else: self.orientation = Orientation.Vertical
        self.point1 = point1
        self.point2 = point2

    # override method
    def get_force_at_point(self, point):
        if self.orientation == Orientation.Horizontal:
            if point.x < min(self.point1.x, self.point2.x) or point.x > max(self.point1.x, self.point2.x): return None
            else: return PointCharge(Point(point.x, self.point1.y), self.intensity).get_force_at_point(point)
        else:
            if point.y < min(self.point1.y, self.point2.y) or point.y > max(self.point1.y, self.point2.y): return None
            else: return PointCharge(Point(self.point1.x, point.y), self.intensity).get_force_at_point(point)

# Charge produced by a group line segments and points. Any point in space is only affected by one of the charges inside the composit. 
class CompositCharge(Charge):
    def __init__(self, line_segment_charges, point_charge, intensity) -> None:
        super().__init__(intensity)
        self.line_segment_charges = line_segment_charges
        self.point_charge = point_charge

    # override method
    def get_force_at_point(self, point):
        for line_segment_charge in self.line_segment_charges:
            force = line_segment_charge.get_force_at_point(point)
            if force != None: return force
        
        return self.point_charge.get_force_at_point(point)