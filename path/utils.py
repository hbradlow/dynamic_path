import numpy as np

def linspace2d(start,end,num):
	return zip(np.linspace(start[0],end[0],num),np.linspace(start[1],end[1],num))

class Path:
	def __init__(self):
		self.points = []
		self.normals = []

	def generate_path(self,length=10,start=np.array([100,300]),end=np.array([700,300])):
		self.points = linspace2d(start,end,length)

	def perturb_path(self,factor=50):
		for index,point in enumerate(self.points):
			r = np.random.rand(2)-np.array([0.5,0.5])
			self.points[index] = point+r*factor

	def get_path_normals(self):
		self.normals = [None for p in self.points]
		for index,point in enumerate(self.points):
			if index > 1:
				xdiff = point[0]-self.points[index-2][0]
				ydiff = point[1]-self.points[index-2][1]
				n = np.array([-ydiff,xdiff])
				n /= np.linalg.norm(n)
				self.normals[index-1] = [n,-n]

class Box:
	def __init__(self,origin=np.array([300,300]),size=np.array([100,100])):
		self.origin = origin
		self.size = size
	def point_in(self,point):
		x = point[0] >= self.origin[0] and point[0] <= self.origin[0]+self.size[0]
		y = point[1] >= self.origin[1] and point[1] <= self.origin[1]+self.size[1]
		return x and y
