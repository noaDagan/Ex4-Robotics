#!/usr/bin/python
#
# load_map.py
#
#
from nav_msgs.msg import Odometry
import rospy
import sys,os
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import math
from tf.transformations import euler_from_quaternion
import tf
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from math import pi

dic_4D = {}

#Dicteonery for the robot directions
class Directions:
	def __init__(self):
		self.up = 'U'
		self.right = 'R'
		self.down = 'D'
		self.left = 'L'
		self.current='U'


def print_map_to_file(grid_by_cells):
	with open('{}/new_grid.txt'.format(os.path.dirname(sys.argv[0])),"w+") as grid_file:
		for row in reversed(grid_by_cells):
			for cell in row:
				grid_file.write("1") if cell else grid_file.write("0")
			grid_file.write("\n")



#The function run DFS algorithm and duild a spening tree in the 4d grid map
def DFS(x, y, grid_4d, robot_size,height,width):
	grid_4d = grid_4d[::-1]
	#temp = height
	#hight=width
	#width = temp
	robot_size_to_grid = robot_size * 2
	open_list = set()
	#Add the first cell
	open_list.add((x,y))
	visited = set()
	neighbor_dict = {}
	direction = []
	#While there are stiil node in the open list
	while open_list:
		#Pop the first cell
		node = open_list.pop()
		node_x = node[0]
		node_y = node[1]
		#If the cell was not viseted
		if node not in visited:
			visited.add(node)
			#Find the cell neighbors
			node_neighbors = find_neighbors(node_x, node_y, height, width, grid_4d)
			#For each neighbor calcute the robot position 
			for node_neighbor in node_neighbors:
				neighbor_x = node_neighbor[0]
				neighbor_y = node_neighbor[1]
				#If the cell was not viseted  and not in the open list
				if node_neighbor not in visited and node_neighbor not in open_list:
					#if cell is not on dictionary create new list with the cell node_neighbor
					if node not in neighbor_dict:
						neighbor_dict[node] = [node_neighbor]
					else:
						neighbor_dict[node].append(node_neighbor)
					#calculate the robot position
					robot_x,robot_y,robot_x_neighbor,robot_y_neighbor =  calculate_robot_position(robot_size,node,node_neighbor,robot_size_to_grid)
					add_to_dir = (robot_x,robot_y,robot_x_neighbor,robot_y_neighbor)
					direction.append(add_to_dir)
					#Add tp the open list
					open_list.add(node_neighbor)
	return neighbor_dict,direction

#Te function calculate the robot position
def calculate_robot_position(robot_size,node,node_neighbor,robot_size_to_grid):
	#Calc the robot x and y
	robot_x = robot_size + ((node[0] - 0)* robot_size_to_grid)
	robot_y = robot_size + ((node[1] - 0)* robot_size_to_grid)
	#Calc the neighbor x and y
	robot_x_neighbor = robot_size + ((node_neighbor[0] - 0) * robot_size_to_grid)
	robot_y_neighbor = robot_size + ((node_neighbor[1] - 0) * robot_size_to_grid)
	return robot_x,robot_y,robot_x_neighbor,robot_y_neighbor
	

#The function check if the node is in the given list
def check_in_list(my_list, node):
	node_x = node[0]
	node_y = node[1]
	#Go over the list
	for element in my_list:
		if element[0] == node_x and element[1] == node_y:
			return True
	return False

#The function find the find_neighbors for a given cell
def find_neighbors(x, y, height, width,grid):

	neighbors = []
	#Check if can go down
	if  x < height - 1:
		if grid[x + 1][y] == 0:
			neighbors.append((x + 1, y))
	##Check if can go up
	if x > 0:
		if grid[x - 1][y] == 0:
			neighbors.append((x - 1, y))
	#Check if can go right
	if y < width - 1:
		if grid[x][y + 1] == 0:
			neighbors.append((x, y + 1))
	#Check if can go left
	if y > 0:
		if grid[x][y - 1] == 0:
			neighbors.append((x,y - 1))
	return neighbors


#the function create a _hemilton cycle on the given grid
def create_hemilton(obstacle, d_grid ,grid_x ,grid_y ,robot_x ,robot_y,robot_size,init_x,init_y,robot_directions):
	path = [(grid_x,grid_y)]
	first = True
	curr_x = grid_x
	curr_y = grid_y
	#While didnt reach the initial position
	while first == True or(grid_x != curr_x or grid_y != curr_y):
		#Check for obstcle in the right
		if check_obs(robot_directions.right,obstacle,curr_x,curr_y,robot_x,robot_y,robot_size,init_x,init_y,robot_directions):
			robot_directions.current = take_step(robot_directions.right,robot_directions)
			curr_x,curr_y = change_step(robot_directions.current,curr_x,curr_y,robot_directions)
		#Check for obstcle in the up
		elif check_obs(robot_directions.up,obstacle,curr_x,curr_y,robot_x,robot_y,robot_size,init_x,init_y,robot_directions):
			robot_directions.current = take_step(robot_directions.up,robot_directions)
			curr_x,curr_y = change_step(robot_directions.current,curr_x,curr_y,robot_directions)
		#Check for obstcle in the left
		elif check_obs(robot_directions.left,obstacle,curr_x,curr_y,robot_x,robot_y,robot_size,init_x,init_y,robot_directions):
			robot_directions.current = take_step(robot_directions.left,robot_directions)
			curr_x,curr_y = change_step(robot_directions.current,curr_x,curr_y,robot_directions)
		path.append((curr_x,curr_y))
		first = False

	return path


# The function calculate the next step direction
def take_step(next_step,robot_directions):
	# continue go up
	if robot_directions.current == robot_directions.up:
		return next_step
	# go down
	if robot_directions.current == robot_directions.right and next_step == robot_directions.right:
		return robot_directions.down
	# go right
	if robot_directions.current == robot_directions.right and next_step == robot_directions.up:
		return robot_directions.right
	# go up
	if robot_directions.current == robot_directions.right and next_step == robot_directions.left:
		return robot_directions.up
	# go left
	if robot_directions.current == robot_directions.right :
		return robot_directions.left
	# go left
	if robot_directions.current == robot_directions.down and next_step == robot_directions.right:
		return robot_directions.left
	# go down
	if robot_directions.current == robot_directions.down and next_step == robot_directions.up:
		return robot_directions.down
	# go right
	if robot_directions.current == robot_directions.down and next_step == robot_directions.left:
		return robot_directions.right
	# go up
	if robot_directions.current == robot_directions.down :
		return robot_directions.up
	# go up
	if robot_directions.current == robot_directions.left and next_step == robot_directions.right:
		return robot_directions.up
	# go left
	if robot_directions.current == robot_directions.left and next_step == robot_directions.up:
		return robot_directions.left
	# go down
	if robot_directions.current == robot_directions.left and next_step == robot_directions.left:
		return robot_directions.down
	# go right
	if robot_directions.current == robot_directions.left :
		return robot_directions.right


#Tje function change the next step accurding to directions
def change_step(next_step,curr_x,curr_y,robot_directions):
	#Check if next step is left
	if next_step == robot_directions.left:
		child_node_x = curr_x 
		child_node_y = curr_y - 1
		return child_node_x ,child_node_y
	#Check if next step is right
	if next_step == robot_directions.right:
		child_node_x = curr_x
		child_node_y = curr_y + 1
		return child_node_x ,child_node_y
	#Check if next step is up
	if next_step == robot_directions.up:
		child_node_x = curr_x + 1
		child_node_y = curr_y 
		return child_node_x ,child_node_y
	child_node_x = curr_x - 1
	child_node_y = curr_y 
	return child_node_x ,child_node_y

#The function check if we have obstacle
def check_obs(robot_direct, obstacle, curr_x, curr_y, robot_x, robot_y,robot_size,init_x,init_y,robot_directions):
	horizional_obs = None
	vertical_obs = None
	horizional_go = None
	vertical_go = None
	#Get the next step
	next_step = take_step(robot_direct,robot_directions)
	#Go up
	if next_step == robot_directions.up:
		node_child_x = curr_x+1
		node_child_y = curr_y 
	#Go right
	if next_step == robot_directions.right:
		node_child_x = curr_x 
		node_child_y = curr_y +1
	#Go left
	if next_step == robot_directions.left:
		node_child_x = curr_x 
		node_child_y = curr_y - 1
	#Go down
	if next_step == robot_directions.down:
		node_child_x = curr_x - 1
		node_child_y = curr_y 
	#For each obstacle
	for obs in obstacle:
		#Calculate the x and y posotion of the node and the chuld node
		curr_robot_x =  robot_x + (curr_x - init_x) * robot_size
		curr_robot_y =  robot_y + (curr_y - init_y) * robot_size
		curr_node_child_x = robot_x + (node_child_x - init_x) * robot_size
		curr_node_child_y = robot_y + (node_child_y - init_y) * robot_size
		#Check if need to go horizional
		if curr_robot_x == curr_node_child_x:
			horizional_go = True
		else:
			horizional_go = False
		#Check if need to go vertical
		if curr_robot_y == curr_node_child_y:
			vertical_go = True
		else:
			vertical_go = False
		#Check if obstacle is horizional
		if obs[0] == obs[2]:
			horizional_obs = True
		else:
			horizional_obs = False
		#Check if obstacle is vertical
		if obs[1] == obs[3]:
			vertical_obs = True
		else:
			vertical_obs = False
		if horizional_go == horizional_obs or vertical_obs == vertical_go:
			ret_value =True
			continue
		minimum_y = None
		maximum_y = None
		minimum_x = None
		maximum_x = None
		minimum_obs = None
		maximum_obs = None
		if vertical_go == True:
			#Get the minimum x between the node and node child
			if curr_robot_x < curr_node_child_x:
				minimum_x = curr_robot_x
				maximum_x = curr_node_child_x
			else:
				minimum_x = curr_node_child_x
				maximum_x = curr_robot_x
			#Get the minimum obs
			if obs[1] < obs[3] :
				minimum_obs =  obs[1]
				maximum_obs =  obs[3]
			else:
				minimum_obs = obs[3]
				maximum_obs =  obs[1]
			#Check for obstacle
			if (minimum_x < obs[0] and obs[0] < maximum_x) and (minimum_obs < curr_robot_y and curr_robot_y < maximum_obs):
				return False
		if vertical_obs == True :
			#Get the minimum y between the node and node child
			if curr_robot_y < curr_node_child_y:
				minimum_y = curr_robot_y
				maximum_y = curr_node_child_y
			else:
				minimum_y = curr_node_child_y
				maximum_y = curr_robot_y
			#Get the minimum obs
			if obs[0] < obs[2] :
				minimum_obs =  obs[0]
				maximum_obs =  obs[2]
			else:
				minimum_obs = obs[2]
				maximum_obs =  obs[0]
			#Check for obstacle
			if (minimum_y < obs[1] and obs[1] < maximum_y) and (minimum_obs < curr_robot_x and curr_robot_x < maximum_obs):
				return False
	return True



def creat_occupancy_grid(my_map,size_of_cell):
	# creating the occupancy grid
	grid = [[None] * my_map.info.width for i in xrange(my_map.info.height)]
	#Fill the grid according to the obstacles in the map
	for i in xrange(my_map.info.height):
		for j in xrange(my_map.info.width):
			#Check for obstacle
			if my_map.data[i * my_map.info.width + j] == 0:
				grid[i][j] = False
			else:
				grid[i][j] = True
	return grid

def create_grid(grid, size_of_cell, height, width):
	global dic_4D
	size_of_cell = int(size_of_cell)
	#Create new hight and width
	new_height = int(math.trunc(height/ size_of_cell))
	new_width = int(math.trunc(width/ size_of_cell))
	new_grid = [[None] * new_width for i in xrange(new_height)]
	count_height = 0
	count_width = 0
	index_width = 0
	#Run over the grid
	for new_i in xrange(new_height):
		for new_j in xrange(new_width):
			is_obs = False
			index_height =int( size_of_cell) * new_i
			count_height = 0
			index_width = int( size_of_cell) * new_j
			dList=list()
			#Run over the inner cell
			for i in xrange(size_of_cell):
				count_width = 0
				for j in xrange(size_of_cell):
					#Addto dicteonery the cells matching the big cell
					if size_of_cell == 2:
						dList.append((index_height + count_height,index_width + count_width))
					if index_height + count_height < height and index_width + count_width < width:
						if grid[index_height + count_height][index_width + count_width]  == True:
							is_obs = True
					count_width += 1
				count_height += 1
			#Check for obstacle
			if is_obs == True:
				new_grid[new_i][new_j] = 1
			else:
				new_grid[new_i][new_j] = 0
			if size_of_cell == 2:
				dic_4D[(new_i,new_j)] = dList
	return new_grid, new_height, new_width

#Class coverade create a hemilton ccycle and mive the robot according to it
class Coverage:
	def __init__(self):
			self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
			rospy.loginfo("Waiting for move_base action server...")
			self.ac.wait_for_server(rospy.Duration(60))
	rospy.loginfo("Connected to move base server")

#The function move the tobot according to the himelton circle
	def start_moving(self,hemilton_path,robot_size):
		rate = rospy.Rate(10)
		node_position = None
		failure = 0
		threshhold = 1
		first = True
		i = 0
		while not rospy.is_shutdown():
			#if didnd reach to the end of the hemilton cycle
			if i < len (hemilton_path):
				#Get the rotation angle
				rotation_angle = self.get_rotation(node_position,hemilton_path[i])
				#Move the robot to the next point in the cycle
				next_node = self.move(hemilton_path[i],rotation_angle,robot_size)
				#Check if the movemend susseded 
				if threshhold <= failure or next_node == True:
					if threshhold <= failure:
						pass
					failure = 0
					#Go to next point int thr cycle
					i = i + 1
					node_position = hemilton_path[i]
				else:
					failure = failure + 1
			rate.sleep()

#The functio return the robot rotation angle according to the current and next node
	def get_rotation(self,node_position,node):
		angle = 180
		#If is first rotate 180
		if node_position == None:
			angle = 180
		#Go straight
		elif node_position[0] < node[0]:
			angle_rotate = 0
		#Turn 180 degrees
		elif node_position[0] > node[0]:
			angle = 180
		#Turn 270 degrees
		elif node_position[1] > node[1]:
			angle = 270
		#Turn 90 degrees
		elif node_position[1] < node[1]:
			angle = 90
		return angle

#The functionrun DFT on the 4d-grid create himelton tn the d-grid and move the robot 
	def start(self):
		robot_directions = Directions()
		robot_size = 0.35
		if rospy.has_param('robot_size'):
			robot_size = rospy.get_param('~robot_size')
		rospy.wait_for_service('static_map')
		try:
			get_static_map = rospy.ServiceProxy('static_map', GetMap)
			response = get_static_map()
			#Get the size of the cell
			size_of_cell= round(robot_size / response.map.info.resolution)
			rospy.loginfo("Received a %d X %d map @ %.3f m/px" %  (response.map.info.width, response.map.info.height, response.map.info.resolution))
			#Create occupancy grid eith the cell size
			grid = creat_occupancy_grid(response.map,size_of_cell)
			#Create the d-grid
			d_grid ,d_height,d_width = create_grid(grid, size_of_cell, response.map.info.height, response.map.info.width)
			print_map_to_file(d_grid)
			size_cell_4d = 2
			#Create 4d-grid
			grid_4D, height_4D, width_4D = create_grid(d_grid, 2, d_height, d_width)
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s" % e)
		listener = tf.TransformListener()
		rate = rospy.Rate(2.0)
		listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(10.0))
		#while not rospy.is_shutdown():
		try:
			resolution = response.map.info.resolution
			#Ger the robot position
			(trans,rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
			robot_x = trans[0]*246
			robot_y =trans[1]*174
			init_x = 6.9
			init_y =5.9
			#Calc the initioal position in the 4d-grid
			grid_x_4d = (robot_x - response.map.info.origin.position.x) / (robot_size *4)
			grid_y_4d = (robot_y - response.map.info.origin.position.y) / (robot_size *4)
			#Calc the initioal position in the d-grid
			grid_x_d = int(math.trunc((robot_x - response.map.info.origin.position.x) / (robot_size*2 )))
			grid_y_d = int(math.trunc((robot_y - response.map.info.origin.position.y) / (robot_size *2)))
			grid_x_4d = int(math.trunc (grid_x_4d))
			grid_y_4d  = int(math.trunc (grid_y_4d ))
			#Run DFS on the 4d-grid
			neighbor_dict, direction = DFS(grid_x_4d , grid_y_4d, grid_4D, robot_size,height_4D,width_4D)
			#Get the hemilton cycle on the d-grid
			hemilton_result = create_hemilton(direction,d_grid,grid_x_d,grid_y_d,init_x,init_y,robot_size,grid_x_d,grid_y_d,robot_directions)
			#Print the cycle to the file
			with open('{}/Coverage_path.txt'.format(os.path.dirname(sys.argv[0])),"w+") as new_file:
				for i in hemilton_result:
					new_file.write("col:"+str(i[0])+" "+"row:"+str(i[1])+"\n")
			#Move the robot according to thr hemilton cycle
			self.start_moving(hemilton_result,robot_size)
			print_map_to_file(grid_4D)

			rospy.loginfo("current position (%f,%f)" % (robot_x, robot_y))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
			rospy.logerr("Service call failed: %s" % e)


	def move(self,node,rotation_angle,robot_size):

		# send a goal to move_base
		goal = MoveBaseGoal()
		# Use the map frame to define goal poses
		goal.target_pose.header.frame_id = 'map'
		# Set the time stamp to "now"
		goal.target_pose.header.stamp = rospy.Time.now()
		# Set x,y positions
		goal.target_pose.pose.position.x = node[0] * robot_size  - 6.9
		goal.target_pose.pose.position.y = node[1] * robot_size - 5.9
		# Calcilate the rad angle
		rotation_rad = rotation_angle * (math.pi/180)
		quat = quaternion_from_euler(0, 0, rotation_rad)
		q_msg = Quaternion(*quat)
		goal.target_pose.pose.orientation = q_msg
		self.ac.send_goal(goal)
		self.ac.wait_for_result()
		# Check for result status
		if self.ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
			return True
		else:
			return False


