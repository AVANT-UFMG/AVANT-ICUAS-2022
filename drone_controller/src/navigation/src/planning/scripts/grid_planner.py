#!/usr/bin/env python
#coding: utf-8

import rospy
from custom_msgs_srvs.msg import Vector3, TrajectoryHandle
from custom_msgs_srvs.srv import Plan, PlanResponse

import numpy as np
import networkx as nx
import cv2

class Planner:
    def __init__(self, map_dim, cell_size):
        self.G = None
        self.graph_dim = None
        self.cell_size = cell_size
        self.map_dim = map_dim

        s = rospy.Service('grid_planner', Plan, self.handler)

    def handler(self, req):
        occ_grid = np.reshape(req.occ_grid.data, (req.occ_grid.info.height, req.occ_grid.info.width))
        occ_grid = self.binarize_map(occ_grid)
        start_pos = [req.start_position.x, req.start_position.y, req.start_position.z]
        goal_pos = [req.goal_position.x, req.goal_position.y, req.goal_position.z]

        self.init_graph(self.cell_size, self.map_dim)
        # occupied_nodes = self.get_occupied_nodes(occ_grid, int(self.cell_size))
        # self.remove_occupied_nodes(occupied_nodes)
        plan = self.plan(start_pos, goal_pos)

        waypoints = []
        for point in plan:
            waypoints.append(Vector3(point[0], point[1], point[2]))
        
        trajectory = TrajectoryHandle(points=waypoints)
        return PlanResponse(success=True, trajectory=trajectory)

    def init_graph(self, cell_size, map_dim):
        map_dim = np.array(map_dim)
        self.graph_dim = tuple(np.floor(map_dim/cell_size).astype(int))
        self.G = nx.grid_graph(dim=self.graph_dim)
        rospy.loginfo("Grid initialized.")

    def binarize_map(self, m, thresh=50):
        m[m < thresh] = 0
        m[m >= thresh] = 100
        return m

    # TODO
    def dilate_map(self, m):
        pass

    def get_occupied_nodes(self, m, cell_size):
        occupied_nodes = []
        for k in range(self.graph_dim[2]):
            for i in range(0, m.shape[0], cell_size):
                for j in range(0, m.shape[1], cell_size):
                    if np.any(m[i:i+cell_size, j:j+cell_size] > 0):
                        occupied_nodes.append((i, j, k))
        return occupied_nodes

    def remove_occupied_nodes(self, occupied_nodes):
        self.G.remove_nodes_from(occupied_nodes)

    def pos_to_node(self, pos, cell_size):
        pos = np.floor(np.array(pos)/cell_size).astype(int)
        pos = tuple(pos)
        return pos

    def node_to_pos(self, node, cell_size):
        pos = np.array(node) * cell_size + cell_size/2
        return pos

    def plan(self, start_pos, goal_pos):
        rospy.loginfo("Started planning...")
        start_node = self.pos_to_node(start_pos, self.cell_size)
        goal_node = self.pos_to_node(goal_pos, self.cell_size)
        path = nx.shortest_path(self.G, source=start_node, target=goal_node)
        waypoints = []
        for p in path:
            waypoints.append(self.node_to_pos(p, self.cell_size))
        return waypoints

rospy.init_node('grid_planner')

map_dim = (25, 15, 4.5)
cell_size = rospy.get_param('grid_planner/cell_size', default=2)
grid_planner = Planner(map_dim, cell_size)

rospy.spin()