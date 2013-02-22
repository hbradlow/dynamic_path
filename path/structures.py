import numpy as np
from scipy import optimize
from path.utils import *
import math
import random
import time

class Environment:
    def __init__(self):
        self.polygons = []
    def add_polygon(self,polygon):
        self.polygons.append(polygon)

def cost(locations,env,initial_location,final_location):
    c = 0
    #initial/final location penalty
    c += 10000*np.linalg.norm(initial_location-np.array(locations[0:2]))
    c += 10000*np.linalg.norm(final_location-np.array(locations[-2:]))
    previous_location = None
    for location in grouper(2,locations):
        location = np.array(location)
        #colision penalty
        for polygon in env.polygons:
            state = State(location)
            d = state.penetration_depth(polygon)
            if d:
                c += 1000*d**2
        #penalty to maintain the lengths of the links
        if previous_location is not None:
            d = abs(np.linalg.norm(location-previous_location)-Path.ideal_length)
            c += d**2
        previous_location = location
    return c

class Path:
    ideal_length = 60.0

    def __init__(self):
        self.states = []

    @property
    def normals(self):
        return [s.normals for s in self.states]
    @property
    def points(self):
        return [s.location for s in self.states]

    """
    def cost(self,env):
        c = 0
        for state in self.states:
            for polygon in env.polygons:
                d = state.penetration_depth(polygon)
                if d:
                    c += d**2
        return c
    """



    def update(self,env,maxiter=5):
        t_init = time.time()

        for index,state in enumerate(self.states):
            if state.normal_points() is not None:
                old_loc = state.location

                x0 = np.array([s.location for s in self.states])
                old_cost = cost(x0.flatten(),env,self.initial_location,self.final_location)

                for point in state.normal_points():
                    x0 = np.array([state.location for state in self.states])
                    x0[index] = point

                    res = optimize.fmin(cost,x0,args=(env,x0[0],x0[-1]),maxiter=maxiter,disp=False)

                    if cost(x0.flatten(),env,self.initial_location,self.final_location) < old_cost:
                        for index,location in enumerate(grouper(2,res)):
                            self.states[index].location = np.array(location)

        
        final_cost = cost(x0.flatten(),env,self.initial_location,self.final_location)
        print "Final cost is",final_cost
        print "Took",time.time()-t_init,"seconds"


    def generate_path(self,length=10,start=np.array([100,300]),end=np.array([700,300])):
        points = linspace2d(start,end,length)
        for p in points:
            self.states.append(State(p))
        self.initial_location = self.states[0].location
        self.final_location = self.states[-1].location

    def perturb_path(self,factor=50):
        for index,point in enumerate(self.points):
            r = np.random.rand(2)-np.array([0.5,0.5])
            self.states[index].location = point+r*factor

    def generate_path_normals(self):
        for index,point in enumerate(self.points):
            if index > 1:
                xdiff = point[0]-self.points[index-2][0]
                ydiff = point[1]-self.points[index-2][1]
                n = np.array([-ydiff,xdiff])
                n /= np.linalg.norm(n)
                self.states[index-1].normals = [n,-n]

class State:
    def __init__(self,location,normals=[]):
        self.location = location
        self.normals = normals
    def normal_points(self,factor=50):
        if self.normals:
            return [
                        self.normals[0]*factor+self.location,
                        self.normals[1]*factor+self.location
                    ]
        else:
            return None
    def penetration_depth(self,polygon):
        if polygon.point_in(self.location):
            return min([edge.distance_to_point(self.location) for edge in polygon.edges])
        else:
            return None

class Segment:
    def __init__(self,start,end):
        self.start = start
        self.end = end
    def length(self):
        return np.linalg.norm(self.end-self.start)
    def distance_to_point(self,point):
        t = (point-self.start).dot(self.end-self.start)/(self.length()**2)
        if t < 0:
            return np.linalg.norm(point-self.start)
        elif t > 1:
            return np.linalg.norm(point-self.end)
        else:
            projection = self.start + t*(self.end-self.start)
            return np.linalg.norm(point-projection)

class Polygon:
    def __init__(self):
        self.edges = []
    def point_in(self,point):
        flag = True
        for edge in self.edges:
            if orient_2d(edge.start,edge.end,point)<0:
                flag = False
        return flag

class Box(Polygon):
    def __init__(self,origin=np.array([300,300]),size=np.array([100,100])):
        self.origin = origin
        self.size = size
        self.edges = [
                        Segment(origin,origin+np.array([size[0],0])),
                        Segment(origin+np.array([size[0],0]),origin+np.array([size[0],size[1]])),
                        Segment(origin+np.array([size[1],size[0]]),origin+np.array([0,size[1]])),
                        Segment(origin+np.array([0,size[1]]),origin),
                    ]
