# class Point:
#     def __init__(self, x, y):
#         self.x = x
#         self.y = y

# class Circle:
#     def __init__(self, x, y, r):
#         self.center = Point(x, y)
#         self.radius = r

# class Rectangle:
#     def __init__(self, x, y, w, h):
#         self.center = Point(x, y)
#         self.width = w
#         self.height = h

# class Square(Rectangle):
#     def __init__(self, x, y, s):
#         self.center = Point(x, y)
#         self.width = s
#         self.height = s

# def intersection_test(circ: Circle, rect: Rectangle) -> bool:
#     Point distance (abs(circle.center.x - rect.center.x), abs(circle.center.y - rect.center.y))

#     # Case: inside rectangle
#     if distance.x <= (rect.width/2): return True
#     if distance.y <= (rect.height/2): return True

#     # Case: far from rectangle
#     if distance.x > (rect.widht/2 + circ.radius): return False
#     if distance.y > (rect.height/2 + circ.radius): return False

#     # Case: corner check
#     cornerDistance = (distance.x - rect.width/2)^2 + (distance.y - rect.height/2)^2
#     return cornerDistance <= (circ.radius^2)

def intersection_test(circle_x, circle_y, circle_r, square_x, square_y, square_s):
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