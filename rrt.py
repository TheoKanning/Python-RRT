from collections import namedtuple
from random import randint, uniform
from math import sqrt
import matplotlib.pyplot as plt
import matplotlib.patches as patches

Node = namedtuple('Node', 'id pos parent')
Region = namedtuple('Region', 'pos width height')  # bottom left x,y

WORLD_SIZE = 100
GOAL_CHANCE = 0.01
velocity = 2


def dist(point1, point2):
    return sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def in_region(point, region):
    return region.pos[0] <= point[0] <= region.pos[0] + region.width \
           and region.pos[1] <= point[1] <= region.pos[1] + region.height


def in_regions(point, obstacles):
    for obstacle in obstacles:
        if in_region(point, obstacle):
            return True
    return False


def get_closest(nodes, point):
    """
    Return the node in the list that's closest to the given point
    """
    return min(nodes, key=lambda x: dist(x.pos, point))


def steer(point1, point2):
    """
    Return an intermediate point on the line between point1 and point2
    """
    total_offset = abs(point2[0] - point1[0]) + abs(point2[1] - point1[1])
    x = point1[0] + (point2[0] - point1[0]) / total_offset
    y = point1[1] + (point2[1] - point1[1]) / total_offset
    return x, y


def calculate_path(start, goal, obstacles):
    nodes = [Node(0, start, 0)]
    while True:
        if uniform(0, 1) < GOAL_CHANCE:
            z_rand = (goal.pos[0] + goal.width / 2, goal.pos[1] + goal.height / 2)
        else:
            z_rand = (randint(0, WORLD_SIZE), randint(0, WORLD_SIZE))
        if in_regions(z_rand, obstacles):
            continue
        nearest = get_closest(nodes, z_rand)
        if z_rand == nearest.pos:
            continue

        new_pos = steer(nearest.pos, z_rand)
        if in_regions(new_pos, obstacles):
            continue
        nodes.append(Node(len(nodes), new_pos, nearest.id))

        if len(nodes) % 100 == 0:
            print("{} Nodes Searched".format(len(nodes)))

        if in_region(new_pos, goal):
            return nodes


if __name__ == '__main__':
    start_pos = (WORLD_SIZE / 4, WORLD_SIZE / 4)
    end_region = Region((3 * WORLD_SIZE / 4, 3 * WORLD_SIZE / 4), 10, 10)
    obstacles = [
        Region([0, 30], 80, 10),
        Region([20, 60], 80, 10)
    ]
    nodes = calculate_path(start_pos, end_region, obstacles)
    node_count = len(nodes)
    print("Nodes Calculated {}".format(node_count))
    fig = plt.figure(1)


    def plot_nodes(nodes, subplot):
        axes = fig.add_subplot(subplot, aspect='equal')
        for node in nodes:
            parent = nodes[node.parent]
            plt.plot([node.pos[0], parent.pos[0]], [node.pos[1], parent.pos[1]], 'k-')

        for obstacle in obstacles:
            axes.add_patch(patches.Rectangle(obstacle.pos, obstacle.width, obstacle.height, color=(0, 0, 0, .3)))

        axes.plot(start_pos[0], start_pos[1], 'ro')
        axes.add_patch(patches.Rectangle(end_region.pos, end_region.width, end_region.height, color='g'))
        axes.set_title("{} Points".format(len(nodes)))
        axes.set_xlim([0, WORLD_SIZE])
        axes.set_ylim([0, WORLD_SIZE])


    nodes.sort(key=lambda n: n.id)
    plot_nodes(nodes[:100], 221)
    plot_nodes(nodes[:500], 222)
    plot_nodes(nodes[:1000], 223)
    plot_nodes(nodes, 224)

    # Highlight path from goal back to start position
    current_node = nodes[-1]
    while current_node.id != 0:
        parent = nodes[current_node.parent]
        plt.plot([current_node.pos[0], parent.pos[0]], [current_node.pos[1], parent.pos[1]], 'b-', linewidth=2.0)
        current_node = parent

    plt.show()
