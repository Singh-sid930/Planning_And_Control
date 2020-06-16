#!/usr/bin/env python


import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection
import matplotlib

import random 
import math


class RRT(object):

	def __init__(self,dim,obstacles,start,goal,robot_r,goal_r,radius,circ_rad):

		self.start = start
		self.goal = goal

		self.Map = {}		
		self.Map['dim'] = dim
		self.Map['obstacles'] = obstacles
		self.Map['start'] = start
		self.Map['goal'] = goal
		self.Map['robot_r'] = robot_r
		self.Map['goal_r'] = goal_r

		self.radius = radius
		self.cir_rad = circ_rad


		self.node_list = []
		self.x_coord = []
		self.y_coord = []
		self.id_list = []
		self.x_coord = np.empty(0)
		self.y_coord = np.empty(0)
		self.x_coord = np.append(self.x_coord,0)
		self.y_coord = np.append(self.y_coord,0)
		self.id_list.append(0)


	def draw_map(self,path,costs):
		fig,ax = plt.subplots()
		fig1,ax1 = plt.subplots()

		ax1.plot(costs)
		ax.set_xlim(-10,10)
		ax.set_ylim(-10,10)
		scat = ax.scatter([self.start[1],self.goal[1],4.108],[self.start[0],self.goal[0],4.075],cmap=matplotlib.cm.spring,c = [0,2,1])
		fig.canvas.draw()
		
		obstacles = self.Map['obstacles']
		plt.pause(2)

		print("Drawing obstacles")

		for obstacle in obstacles:
			# print(obstacle)
			# print("coords=",(obstacle[1,1],obstacle[0,0]))
			# print("width = ", abs((obstacle[0,0])-(obstacle[1,0])))
			# print("height = ", abs(obstacle[0,1]-obstacle[1,1]))
			rect = patches.Rectangle((obstacle[0,0],obstacle[1,1]),abs((obstacle[0,0])-(obstacle[1,0])),abs(obstacle[0,1]-obstacle[1,1]),linewidth=1.0,edgecolor='r',facecolor='r')
			ax.add_patch(rect)
		
		print("drawing connected points")
		lines = []
		steps = 0 
		for node in self.node_list:
			self_coord = node['coord']
			parent_id = node['parent']
			parent_coord = self.node_list[parent_id]['coord']
			line = [(self_coord[0],self_coord[1]),(parent_coord[0],parent_coord[1])]
			lines.append(line)
		
		lc = LineCollection(lines,colors = 'b',linewidths = 0.2)
		ax.add_collection(lc)
		scat.axes.figure.canvas.draw_idle()

		plt.pause(10)

		lines_path = []

		for i in range(len(path)-1):
			point = path[i]
			next_point = path[i+1]
			line = [(point[0],point[1]),(next_point[0],next_point[1])]
			lines_path.append(line)

		lc_path = LineCollection(lines_path,colors = 'r',linewidths = 0.8)
		ax.add_collection(lc_path)
		scat.axes.figure.canvas.draw_idle()
		ax.set_xlim(-10,10)
		ax.set_ylim(-10,10)
		plt.pause(60)
		

	def FindPath(self):
		node = {}
		node["id"] = 0
		node['coord'] = self.Map['start']
		node['parent'] = 0
		node['cost'] = 0 	
		node['traj'] = [0,0]
		self.node_list.append(node)

		optimum_cost = 40

		costs = []

		i = 0
		
		while(True):

			x = self.sample()

			if(self.is_in_collision(x)):
				continue
			
			x,parent_id,parent_coord = self.connect(x)

			if(self.is_in_collision(x)):
				continue

			if(self.path_is_in_collision(x,parent_coord)):
				continue
	

			i+=1
			
			node = {}
			node["id"] = i
			self.id_list.append(i)


			node['coord'] = x
			self.x_coord = np.append(self.x_coord,x[0])
			self.y_coord = np.append(self.y_coord,x[1])
			node['parent'] = parent_id

			parent_cost = self.node_list[parent_id]['cost']
			cost, traj = self.steer(x,parent_coord,parent_cost)
			node['cost'] = cost
			node['traj'] = traj

			self.node_list.append(node)

			self.rewire(x,i,parent_id)


			if((abs(x[0]-self.Map['goal'][0])<self.Map['goal_r']) and (abs(x[1]-self.Map['goal'][1])<self.Map['goal_r'])):

				if(cost<optimum_cost):
					print("new optimum cost is found")
					optimum_cost = cost
					final_id = i
				print("a path has been found")
				print("number of steps taken are : ", i)

			costs.append(optimum_cost)
				
			if(i>10000):
				self.print_path(final_id,costs)
				break

	def print_path(self,final_id,costs):
		i = final_id
		r = 0
		path = []
		while(self.node_list[i]["id"] !=0.0):
			path.append(self.node_list[i]["coord"])
			i = self.node_list[i]["parent"]
		path.append([0,0])
		# print(path)
		self.draw_map(path,costs)



	

	def path_is_in_collision(self,x1,x2):
		m = (x2[1]-x1[1])/(x2[0]-x1[0])
		y_1 = x1[1]
		x_1 = x1[0]
		for obstacle in self.Map['obstacles']:
			x_range = np.arange(x1[0],x2[0],0.01)
			for x in x_range:
				y = m*(x-x_1) + y_1
				if(obstacle[0,0]-0.6<=x<=obstacle[1,0]+0.6):
					if(obstacle[1,1]-0.6<= y<=obstacle[0,1]+0.6):
						return True

	
	def sample(self):
		x = np.random.uniform(self.Map['dim'][0,0],self.Map['dim'][0,1])
		y = np.random.uniform(self.Map['dim'][1,0],self.Map['dim'][1,1])
		return [x,y]

	def steer(self,x1,x2,parent_cost):
		x_rad = x1[0] - x2[0]
		y_rad = x1[1] - x2[1]
		theta = np.arctan2(y_rad,x_rad)

		cost = parent_cost + np.sqrt(math.pow(x_rad,2)+math.pow(y_rad,2))
		steer = [math.cos(theta),math.sin(theta)]
		return cost,steer
		

	def is_in_collision(self,x):
		for obstacle in self.Map['obstacles']:
			# print(obstacle)
			# print(x)
			# print("comparing {} to lie between {} and  {}".format(x[0],obstacle[0,0],obstacle[1,0]))
			if(obstacle[0,0]-0.6<=x[0]<=obstacle[1,0]+0.6):
				# print("comparing {} to lie between {} and  {}".format(x[1],obstacle[1,1],obstacle[0,1]))
				if(obstacle[1,1]-0.6 <= x[1]<=obstacle[0,1]+0.6):
					# print(obstacle)
					# print(x[0])
					# print(x[1])
					# exit(1)
					return True

			
		return False


	def nearest(self,x):

		dist_x = self.x_coord - x[0]
		dist_y = self.y_coord - x[1]

		euc_distances = np.sqrt(np.power(dist_x,2)+np.power(dist_y,2))

		near_ind = np.argmin(euc_distances)

		parent_coord = self.node_list[near_ind]['coord']

		parent_id = near_ind

		return parent_id,parent_coord
		

	def connect(self,x):

		parent_id,parent_coord = self.nearest(x)

		rad_x = x[0] - parent_coord[0]
		rad_y = x[1] - parent_coord[1]

		dist = np.sqrt(math.pow(rad_x,2)+math.pow(rad_y,2))

		#####################################################

		N = len(self.node_list)

		self.radius = 5*(np.log(N)/N)**0.5

		if self.radius>2.0:
			self.radius = 2.0

		# print("computed radius is : " , self.radius)

		######################################################

		if(dist>self.radius):

			theta = np.arctan2(rad_y,rad_x)
			x[0] = self.radius*math.cos(theta) + parent_coord[0]
			x[1] = self.radius*math.sin(theta) + parent_coord[1]

		return x,parent_id,parent_coord

	def rewire(self,x,curr_id,parent_id):

		dist_x = self.x_coord - x[0]
		dist_y = self.y_coord - x[1]

		euc_distances = np.sqrt(np.power(dist_x,2)+np.power(dist_y,2))

		# ################################################

		N = len(self.node_list)

		self.cir_rad = 5*(np.log(N)/N)**0.5

		if self.radius>2.0:
			self.radius = 2.0

		# #################################################



		in_rad_ind = np.where(euc_distances<=self.cir_rad)

		cost_curr_node = self.node_list[curr_id]['cost']

		cost_curr_parent = self.node_list[parent_id]['cost']

		for i in range(len(in_rad_ind[0])):

			check_id = in_rad_ind[0][i]

			if(check_id == curr_id):
				continue

			check_cost = self.node_list[check_id]['cost']

			if(check_cost<cost_curr_parent):
				x_curr = self.node_list[curr_id]['coord']
				x_parent = self.node_list[check_id]['coord']
				if(self.path_is_in_collision(x_curr,x_parent)):
					continue
				self.node_list[curr_id]['parent'] = check_id
				cost_curr_parent = check_cost


		# for i in range(len(in_rad_ind[0])):
		# 	check_id = in_rad_ind[0][i]
		# 	if(check_id == curr_id):
		# 		continue
		# 	check_parent_cost = self.node_list[self.node_list[check_id]['parent']]['cost']

		# 	if(cost_curr_node<check_parent_cost):
		# 		self.node_list[check_id]['parent'] = curr_id
		# 		# cost_curr_parent = check_cost


		


if __name__ == '__main__':
	dim = np.array([[-10.0,10.0],[-10.0,10.0]])
	obstacles = np.array([[[-6.0,-4.0],[0.0,-5.0]],[[4.0,9.0],[5.0,-4.0]]])
	start = np.array([0,0])
	goal = np.array([8,8])
	robot_r = 0.6
	goal_r = 0.5
	radius = 1.0
	circ_rad = 1.0
	rrt_obj = RRT(dim,obstacles,start,goal,robot_r,goal_r,radius,circ_rad)
	rrt_obj.FindPath()
