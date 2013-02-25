import numpy as np
from scipy import optimize
from path.utils import *
import math
import random
import time
import copy

MAX_ITER_DEFAULT = 5

class Environment:
    def __init__(self):
        self.polygons = []
    def add_polygon(self,polygon):
        self.polygons.append(polygon)
    def point_in(self,point):
        flag = False
        for polygon in self.polygons:
            if polygon.point_in(point):
                flag = True
        return flag

def cost(locations,env,initial_location,final_location):
    """
        Calculate the cost of a path
    """
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
                c += 100000+1000*d**2

        #penalty to maintain the lengths of the links
        if previous_location is not None:
            d = abs(np.linalg.norm(location-previous_location)-Path.ideal_length)
            c += 100*d**2
        previous_location = location

    return c

class Path:
    ideal_length = 60.0

    def __init__(self):
        self.states = []
        self.factor = 50
        self.discount = .8
        self.save()

    @property
    def normals(self):
        return [s.normals for s in self.states]
    @property
    def points(self):
        return [s.location for s in self.states]

    def benchmark(self,env,iterations=10,draw_callback=None,quite=False):
        """
            Run the dynamic and simple optimization and calculate show their final costs.
        """

        self.save()

        cost = None

        if not quite:
            print "Trying dynamic with before_movement"
        for i in range(iterations):
            cost = self.update(env,dynamic=True,quite=True,before_movement=True)
            if draw_callback:
                draw_callback()
        if not quite:
            print "Dynamic final cost (before_movement):",cost

        self.reset()

        if not quite:
            print "Trying dynamic without before_movement"
        for i in range(iterations):
            cost = self.update(env,dynamic=True,quite=True)
            if draw_callback:
                draw_callback()
        if not quite:
            print "Dynamic final cost (not before_movement):",cost

        self.reset()

        if not quite:
            print "Trying simple"
        for i in range(iterations):
            cost = self.update(env,dynamic=False,quite=True)
            if draw_callback:
                draw_callback()
        if not quite:
            print "Simple final cost:",cost

    def save(self):
        """
            Save a copy of the current path
        """
        self.saved_states = copy.deepcopy(self.states)
        self.saved_factor = copy.deepcopy(self.factor)
        self.saved_discount = copy.deepcopy(self.discount)

    def reset(self):
        """
            Reset the path to the previously saved state
        """
        self.states = self.saved_states
        self.discount = self.saved_discount
        self.factor = self.saved_factor
        self.save()

    def simple_update(self,env,maxiter=MAX_ITER_DEFAULT,optimize_callback=None):
        """
            Simply uses scipy fmin to optimize the path.
        """
        x0 = np.array([s.location for s in self.states])
        if optimize_callback:
            res = grouper(2,optimize_callback(x0))
        else:
            res = x0
        for index,location in enumerate(res):
            self.states[index].location = np.array(location)
            self.generate_path_normals()
        
    def dynamic_update(self,env,maxiter=MAX_ITER_DEFAULT,optimize_callback=None,before_movement=False):
        for index,state in enumerate(self.states): #loop over all the states
            if state.normal_points() is not None:

                x0 = np.array([s.location for s in self.states])
                old_cost = cost(x0.flatten(),env,self.initial_location,self.final_location)

                for point in state.normal_points(factor=self.factor):

                    x0[index] = point #try moving the state in the direction of one of the normals

                    if before_movement:
                        if optimize_callback:
                            res = grouper(2,optimize_callback(x0))
                            x0 = np.array([location for location in res])
                        else:
                            res = x0

                    # if moving in the direction of this normal helped, then update the path
                    if cost(x0.flatten(),env,self.initial_location,self.final_location) < old_cost:
                        if not before_movement:
                            if optimize_callback:
                                res = grouper(2,optimize_callback(x0))
                            else:
                                res = x0
                        for index,location in enumerate(res):
                            self.states[index].location = np.array(location)
                            self.generate_path_normals()

    def update(self,env,maxiter=MAX_ITER_DEFAULT,quite=False,dynamic=True,**kwargs):
        """
            Optimize the path.
        """
        t_init = time.time()

        def optimize_callback(x0):
            res = optimize.fmin(cost,
                                x0,
                                args=(env,self.initial_location,self.final_location),
                                maxiter=maxiter,
                                disp=False)
            return res
        if dynamic:
            self.dynamic_update(env,maxiter,optimize_callback=optimize_callback,**kwargs)
        else:
            self.simple_update(env,maxiter,optimize_callback=optimize_callback,**kwargs)


        x0 = np.array([s.location for s in self.states])
        final_cost = cost(x0.flatten(),env,self.initial_location,self.final_location)

        if not quite:
            print "Final cost is",final_cost
            print "Took",time.time()-t_init,"seconds"

        self.factor *= self.discount
        return final_cost


    def generate_path(self,length=15,start=np.array([100,300]),end=np.array([700,300])):
        """
            Generate a path from start to end.
        """
        Path.ideal_length = np.linalg.norm(start-end) / length # the desired length of the segments

        points = linspace2d(start,end,length)
        for p in points:
            self.states.append(State(p))

        #save the initial and final locations for use as costs
        self.initial_location = self.states[0].location
        self.final_location = self.states[-1].location

    def perturb_path(self,factor=10):
        """
            Randomly perturb the path.
        """
        for index,point in enumerate(self.points):
            r = np.random.rand(2)-np.array([0.5,0.5])
            self.states[index].location = point+r*factor

    def generate_path_normals(self):
        """
            Calculate the normals of the states in the path.
        """
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
                        Segment(origin+np.array([size[0],size[1]]),origin+np.array([0,size[1]])),
                        Segment(origin+np.array([0,size[1]]),origin),
                    ]
