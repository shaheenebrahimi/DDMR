import numpy as np
import math
import yaml

MAP_OBSTACLE = "#"
MAP_FREE = 0

# --------------------------------------
# Vector Util
# --------------------------------------

def distance(x0, y0, x1, y1):
   return np.linalg.norm(np.array([x0-x1,y0-y1]))

def magnitude(vector):
   return np.sqrt(np.dot(np.array(vector), np.array(vector)))

def norm(vector):
   return np.array(vector)/magnitude(np.array(vector))

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


# --------------------------------------
# Map Util
# --------------------------------------

def load_world(file_name):
   '''
   Reads a file that describes a 2D grid world and returns a dictionary describing the properties of that robot.
   '''
   with open(file_name) as f:
      world = yaml.safe_load(f)
   world['map_str'] = world['map']
   data, obstacles, width, height, map_2d = string_to_map(world['map'])
   world['map'] = data
   world['obstacles'] = obstacles
   world['width'] = width
   world['height'] = height
   world['map_2d'] = map_2d
   return world

def string_to_map(map_str):
   data = []
   obstacles = []

   map_2d = [s for s in map_str.split('\n')[::-1] if s != ""]

   for row, line in enumerate(map_2d):
      for col, char in enumerate(line):
         if char == '#':
               data.append(100)
               obstacles.append((row, col))
         elif char == '.':
               data.append(0)

   height = len(map_2d)
   width = int(len(data) / height)

   return data, obstacles, width, height, map_2d


# --------------------------------------
# Collision Util
# --------------------------------------

def circle_intersection_test(circle_x, circle_y, circle_r, square_x, square_y, square_s):
   '''
   Intersection test between circle and square
   '''
   dist_x = abs(circle_x - square_x)
   dist_y = abs(circle_y - square_y)

   # Case: far from rectangle
   if dist_x > (square_s/2 + circle_r): return False
   if dist_y > (square_s/2 + circle_r): return False

   # Case: center inside rectangle
   if dist_x <= (square_s/2): return True
   if dist_y <= (square_s/2): return True

   # Case: corner check
   cornerDistance = (dist_x - square_s/2)**2 + (dist_y - square_s/2)**2
   return cornerDistance <= (circle_r**2)

def line_intersection_test(line_x0, line_y0, line_x1, line_y1, square_x, square_y, square_s):
   '''
   Intersection test between line segment and square
   '''
   # Line equation
   def f(x):
      return ((line_y1 - line_y0) / (line_x1 - line_x0)) * (x - line_x0) + line_y0
   
   # Get bounds
   square_x_min = square_x - square_s/2
   square_x_max = square_x + square_s/2
   square_y_min = square_y - square_s/2
   square_y_max = square_y + square_s/2

   # False if NEITHER square x are in interval
   x_interval_min = min(line_x0, line_x1)
   x_interval_max = max(line_x0, line_x1)
   if not (square_x_min >= x_interval_min and square_x_min <= x_interval_max) and not (square_x_max >= x_interval_min and square_x_max <= x_interval_max):
      return False

   # Ray left side test
   left_intersection_y = f(square_x_min)
   if left_intersection_y <= square_y_max and left_intersection_y >= square_y_min:
      return True

   # Ray right side test
   right_intersection_y = f(square_x_max)
   if right_intersection_y <= square_y_max and right_intersection_y >= square_y_min:
      return True

   return False

def point_intersection_test(x, y, square_x, square_y, square_s):
   # Check bounds
   square_x_min = square_x - square_s/2
   square_x_max = square_x + square_s/2
   square_y_min = square_y - square_s/2
   square_y_max = square_y + square_s/2
   return x >= square_x_min and x <= square_x_max and y >= square_y_min and y <= square_y_max
