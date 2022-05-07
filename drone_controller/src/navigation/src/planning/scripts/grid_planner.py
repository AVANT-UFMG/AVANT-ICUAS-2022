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
        # map preprocessing
        m = np.reshape(req.occ_grid.data, (req.occ_grid.info.height, req.occ_grid.info.width))
        bin_m = self.binarize_map(m)
        dil_m = self.dilate_map(bin_m)

        start_pos = [req.start_position.x, req.start_position.y, req.start_position.z]
        goal_pos = [req.goal_position.x, req.goal_position.y, req.goal_position.z]

        self.init_graph(self.cell_size, self.map_dim)
        self.remove_occupied_nodes(dil_m, self.cell_size)
        plan = self.plan(start_pos, goal_pos)

        waypoints = []
        for point in plan:
            waypoints.append(Vector3(point[0], point[1], point[2]))
        
        trajectory = TrajectoryHandle(points=waypoints)
        return PlanResponse(success=True, trajectory=trajectory)

    def init_graph(self, cell_size, map_dim):
        map_dim = np.array(map_dim)
        self.graph_dim = tuple(np.floor(map_dim/cell_size).astype(int))
        self.G = nx.grid_graph(dim=self.graph_dim[::-1])
        rospy.loginfo("Grid initialized.")

    def binarize_map(self, m, thresh=50):
        m[m < thresh] = 0
        m[m >= thresh] = 100
        return m

    def dilate_map(self, m, k_size=5):
        kernel = np.ones((k_size, k_size))
        m_img = (m/100 * 255).astype(np.uint8)
        m_img = cv2.dilate(m_img, kernel)
        return m_img/255 * 100

    def remove_occupied_nodes(self, m, cell_size):
        rospy.loginfo('Removing occupied nodes from grid graph.')
        total_nodes = self.G.number_of_nodes()
        step = m.shape[0]/self.map_dim[0] * cell_size
        for k in range(self.graph_dim[2]):
            for j in range(self.graph_dim[1]):
                for i in range(self.graph_dim[0]):
                    ri, rf = int(j*step), int((j+1)*step)
                    ci, cf = int(i*step), int((i+1)*step)
                    if np.any(m[ri:rf, ci:cf] > 0):
                        try:
                            self.G.remove_node((i, j, k))
                            rospy.logdebug('Removed node {}'.format((i, j, k)))
                        except nx.NetworkXError as e:
                            rospy.logwarn('Could not remove node {} because: {}'.format((i, j, k), str(e)))
        print(self.G.nodes())
        rospy.loginfo('{} nodes removed.'.format(total_nodes - self.G.number_of_nodes()))

    def pos_to_node(self, pos, cell_size):
        pos = [pos[0] + self.map_dim[0]/2, pos[1] + self.map_dim[1]/2, pos[2]]
        pos = np.floor(np.array(pos)/cell_size).astype(int)
        pos = tuple(pos)
        return pos

    def node_to_pos(self, node, cell_size):
        pos = np.array(node) * cell_size + cell_size/2 - np.array([self.map_dim[0]/2, self.map_dim[1]/2, 0])
        return pos

    def plan(self, start_pos, goal_pos):
        rospy.loginfo("Started planning...")
        start_node = self.pos_to_node(start_pos, self.cell_size)
        goal_node = self.pos_to_node(goal_pos, self.cell_size)
        path = nx.shortest_path(self.G, source=start_node, target=goal_node)
        rospy.loginfo('Planning is done.')
        waypoints = []
        for p in path:
            waypoints.append(self.node_to_pos(p, self.cell_size))
        print(waypoints)
        return waypoints
        

rospy.init_node('grid_planner')

map_dim = (25, 15, 4.5)
cell_size = rospy.get_param('grid_planner/cell_size', default=2)
grid_planner = Planner(map_dim, cell_size)

rospy.spin()