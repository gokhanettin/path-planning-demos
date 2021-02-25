import math
import random
import copy
import matplotlib.pyplot as plt
import numpy as np

class RRTStar:
    """
    Class for RRT Star planning
    """

    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0.0

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.obstacle_list = obstacle_list
        self.node_list = [self.start]
        self.path = None
        self.anim = {}
        self.goal_node = self.Node(goal[0], goal[1])

    def step(self):
        # 1 - Get a random node, occasionally select the goal in place of
        # random node.
        random_node = self.get_random_node()
        # 2 - Find existing nearest node to the random node
        nearest_ind = self.get_nearest_node_index(self.node_list, random_node)
        nearest_node = self.node_list[nearest_ind]

        # 3 - Find the node that expands towards the random node from the
        # nearest node with max of given extand distance
        new_node = self.steer(self.node_list[nearest_ind], random_node,
                                self.expand_dis)

        # 4 - Compute the cost to reach at this new node from the starting node
        new_node.cost = nearest_node.cost + \
            math.hypot(new_node.x-nearest_node.x,
                        new_node.y-nearest_node.y)

        # 5 - If no collision, we don't connect the new node to the nearest of
        # the random node, but we select a nearest node within a circle
        # centered at new node with radius expand distance or we apply some
        # dynamic radius such that the circle gets smaller as we add nodes.
        # In other words, new node changes its parent.

        # 6- We also rewire the nodes within the circle if the new node can
        # reduce their costs for coming from the starting point.
        if self.check_collision(new_node, self.obstacle_list):
            near_inds = self.find_near_nodes(new_node)
            node_with_updated_parent = self.choose_parent(
                new_node, near_inds)
            if node_with_updated_parent:
                self.rewire(node_with_updated_parent, near_inds)
                self.node_list.append(node_with_updated_parent)
            else:
                self.node_list.append(new_node)

        # 7- Check if there is a path from a node to the goal
        # without a collision.
        last_index = self.search_best_goal_node()
        if last_index is not None:
            self.path = self.generate_final_course(last_index)


        self.anim["random_node"] = random_node
        self.anim["nearest_node"] = nearest_node
        self.anim["new_node"] = new_node

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node
            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.obstacle_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.expand_dis
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree
                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.
        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.obstacle_list)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

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

    def get_anim_data(self):
        return self.anim

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
        fig, ax = plt.subplots()
        ax.set_title("RRT*")

        obstacle_list = [
            (5, 5, 1),
            (3, 6, 2),
            (3, 8, 2),
            (3, 10, 2),
            (7, 5, 2),
            (9, 5, 2),
            (8, 10, 1),
            (6, 12, 1),
        ]  # [x,y,size(radius)]

        # Set Initial parameters
        initial_rrtstar = RRTStar(
            start=[0, 0],
            goal=[6.0, 10.0],
            rand_area=[-2, 15],
            obstacle_list=obstacle_list)

        self.rrtstar_list = [initial_rrtstar]
        self.current_index = 0
        fig.canvas.mpl_connect('key_press_event', self.step)
        self.initial_plot(initial_rrtstar)

    def step(self, event):
        rrtstar = None
        if event.key == "right":
            self.current_index += 1
            if self.current_index < len(self.rrtstar_list):
                rrtstar = self.rrtstar_list[self.current_index]
            elif self.current_index == len(self.rrtstar_list):
                rrtstar = copy.deepcopy(self.rrtstar_list[-1])
                rrtstar.step()
                self.rrtstar_list.append(rrtstar)
        elif event.key == "left":
           self.current_index -= 1
           if self.current_index >= 0:
               rrtstar = self.rrtstar_list[self.current_index]

        if rrtstar:
            self.plot(rrtstar)
            event.canvas.draw()

    def initial_plot(self, rrtstar):
        plt.axis("equal")
        plt.grid(True)
        plt.axis([-2, 15, -2, 15])

        plt.plot(rrtstar.start.x, rrtstar.start.y, "xr")
        plt.plot(rrtstar.end.x, rrtstar.end.y, "xr")

        for (ox, oy, size) in rrtstar.obstacle_list:
            circle = plt.Circle((ox, oy), size, color="b")
            plt.gca().add_patch(circle)

    def plot(self, rrtstar):
        anim = rrtstar.get_anim_data()
        if not anim:
            return

        plt.clf()
        self.initial_plot(rrtstar)
        random_node = anim["random_node"]
        plt.plot(random_node.x, random_node.y, "^k")

        new_node = anim["new_node"]
        circle = plt.Circle((new_node.x, new_node.y), rrtstar.expand_dis,
                            fill=False)
        plt.gca().add_patch(circle)

        for node in rrtstar.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        if rrtstar.path:
            plt.plot([x for (x, y) in rrtstar.path],
                     [y for (x, y) in rrtstar.path], '-r')


sa = StepAnimation()
plt.show()
