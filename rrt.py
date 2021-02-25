import math
import random
import copy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class RRT:
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500):
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = [self.start]
        self.path = None
        self.anim = {}

    def step(self):
        # 1 - Get a random node, occasionally select the goal in place of
        # random node.
        random_node = self.get_random_node()
        # 2 - Find existing nearest node to the random node
        nearest_ind = self.get_nearest_node_index(self.node_list, random_node)
        nearest_node = self.node_list[nearest_ind]
        # 3 - Expand towards the random node from the nearest node with max of
        # given extand distance
        new_node = self.steer(nearest_node, random_node, self.expand_dis)

        # 4 - If no collision add the new expand node to the list.
        if self.check_collision(new_node, self.obstacle_list):
            self.node_list.append(new_node)

        # 5- If the goal is within the expand distance from the last node,
        # directly expand towards the goal node. If no collision, the path is
        # found.
        if self.calc_dist_to_goal(self.node_list[-1].x,
                                    self.node_list[-1].y) <= self.expand_dis:
            final_node = self.steer(self.node_list[-1], self.end,
                                    self.expand_dis)
            if self.check_collision(final_node, self.obstacle_list):
                self.path = self.generate_final_course(len(self.node_list) - 1)

        # Save some internals for animation
        self.anim["random_node"] = random_node
        self.anim["nearest_node"] = nearest_node
        self.anim["new_node"] = new_node

    def get_anim_data(self):
        return self.anim

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        path.reverse()
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_collision(node, obstacle_list):
        if node is None:
            return False

        for (ox, oy, size) in obstacle_list:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


class StepAnimation:
    def __init__(self):
        self.fig, ax = plt.subplots()
        ax.set_title("RRT")

        obstacle_list = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                        (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
        # Set Initial parameters
        initial_rrt = RRT(
            start=[0, 0],
            goal=[6.0, 10.0],
            rand_area=[-2, 15],
            obstacle_list=obstacle_list)

        self.rrt_list = [initial_rrt]
        self.current_index = 0
        self.fig.canvas.mpl_connect('key_press_event', self.step)
        self.initial_plot(initial_rrt)

    def step(self, event):
        rrt = None
        if event.key == "right":
            self.current_index += 1
            if self.current_index < len(self.rrt_list):
                rrt = self.rrt_list[self.current_index]
            elif self.current_index == len(self.rrt_list):
                rrt = copy.deepcopy(self.rrt_list[-1])
                rrt.step()
                self.rrt_list.append(rrt)
        elif event.key == "left":
           self.current_index -= 1
           if self.current_index >= 0:
               rrt = self.rrt_list[self.current_index]

        if rrt:
            self.plot(rrt)
            event.canvas.draw()

    def initial_plot(self, rrt):
        plt.axis("equal")
        plt.grid(True)
        plt.axis([-2, 15, -2, 15])

        plt.plot(rrt.start.x, rrt.start.y, "xr")
        plt.plot(rrt.end.x, rrt.end.y, "xr")

        for (ox, oy, size) in rrt.obstacle_list:
            deg = list(range(0, 360, 5))
            deg.append(0)
            xl = [ox + size * math.cos(np.deg2rad(d)) for d in deg]
            yl = [oy + size * math.sin(np.deg2rad(d)) for d in deg]
            plt.plot(xl, yl, "-b")

    def plot(self, rrt):
        anim = rrt.get_anim_data()
        if not anim:
            return

        plt.clf()
        self.initial_plot(rrt)
        random_node = anim["random_node"]
        plt.plot(random_node.x, random_node.y, "^k")

        for node in rrt.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        if rrt.path:
            plt.plot([x for (x, y) in rrt.path],
                     [y for (x, y) in rrt.path], '-r')


sa = StepAnimation()
plt.show()

