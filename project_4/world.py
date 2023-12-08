# The load_world method reads a file that describes a 2D grid world
# and returns a dictionary describing the properties of that robot.

import yaml

def load_world(file_name):
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
