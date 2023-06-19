from enum import Enum
from queue import PriorityQueue
import numpy as np
import cv2

# Quadroter assume all actions cost the same.


class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1, 180)
    EAST = (0, 1, 1, 0)
    NORTH = (-1, 0, 1, 90)
    SOUTH = (1, 0, 1, -90)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])

    @property
    def yaw(self):
        return self.value[3]
    


class A_star:
    def valid_actions(self, grid, current_node):
        """
        Returns a list of valid actions given a grid and current node.
        """
        valid_actions = list(Action)
        n, m = grid.shape[0] - 1, grid.shape[1] - 1
        x, y = current_node

        # check if the node is off the grid or
        # it's an obstacle

        if x - 1 < 0 :
            valid_actions.remove(Action.NORTH)
        elif( grid[x - 1, y] == 1):
            valid_actions.remove(Action.NORTH)
        if x + 1 > n :
            valid_actions.remove(Action.SOUTH)
        elif( grid[x + 1, y] == 1):
            valid_actions.remove(Action.SOUTH)
        if y - 1 < 0 :
            valid_actions.remove(Action.WEST)
        elif( grid[x, y - 1] == 1):
            valid_actions.remove(Action.WEST)
        if y + 1 > m :
            valid_actions.remove(Action.EAST)
        elif( grid[x, y + 1] == 1):
            valid_actions.remove(Action.EAST)

        return valid_actions

    def heuristic_func(self, position, goal_position):
        # TODO: write a heuristic!
        h = abs(position[0] - goal_position[0]) + \
            abs(position[1] - goal_position[1])
        return h

    def a_star(self, grid, start, goal):
        """
        Given a grid and heuristic function returns
        the lowest cost path from start to goal.
        """

        path = []
        path_cost = 0
        queue = PriorityQueue()
        queue.put((0, start))
        visited = set(start)

        branch = {}
        found = False

        while not queue.empty():
            item = queue.get()
            current_cost = item[0]
            current_node = item[1]

            if current_node == goal:
                print('Found a path.')
                found = True
                break
            else:
                # Get the new vertexes connected to the current vertex
                for a in self.valid_actions(grid, current_node):
                    next_node = (
                        current_node[0] + a.delta[0], current_node[1] + a.delta[1])
                    new_cost = current_cost + a.cost + \
                        self.heuristic_func(next_node, goal)

                    if next_node not in visited:
                        visited.add(next_node)
                        queue.put((new_cost, next_node))

                        branch[next_node] = [new_cost, current_node, a]

        if found:
            # retrace steps
            n = goal
            path_cost = branch[n][0]
            path.append([branch[n][1][0] -start[0] - 0.5, branch[n][1][1] - start[1] + 0.5, branch[n][2].yaw])
            while branch[n][1] != start:
                path.append([branch[n][1][0] -start[0] - 0.5, branch[n][1][1] - start[1] + 0.5, branch[n][2].yaw])
                n = branch[n][1]
            path.append([branch[n][1][0] -start[0] - 0.5, branch[n][1][1] - start[1] + 0.5, branch[n][2].yaw])
            path.append([0, 0, 0])

        return path[::-1], path_cost

    def point(self, p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_check(self, p1, p2, p3, epsilon=1e-6):
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    def prune_path(self, grid, start, goal):
        path, path_cost = self.a_star(grid, start, goal)
        pruned_path = [p for p in path]
        # TODO: prune the path!
        for i in range(0, len(path) - 2):
            p1 = self.point(path[i][:2])
            p2 = self.point(path[i+1][:2])
            p3 = self.point(path[i+2][:2])
            if self.collinearity_check(p1, p2, p3):
                pruned_path.remove(path[i+1])
        return pruned_path

    def create_grid(self, img_path):
        img = cv2.imread(img_path, 0)
        img_reverted = cv2.bitwise_not(img)
        new_img = img_reverted / 255.0
        final_img = np.where(new_img < 0.2, 0, 1)
        return final_img


# if __name__ == '__main__':
#     img_path = "/home/swadine/catkin_ws/src/a_star_gnc/worlds/test_world.pgm"
#     object = A_star()
#     start = (13, 8)
#     goal = (1, 18)
#     grid = object.create_grid(img_path)
#     print(grid)
#     print(grid.shape)
#     path = object.prune_path(grid, start, goal)
#     print(path)
