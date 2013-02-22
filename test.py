import Tkinter as tk
import numpy as np
from path import utils
from path import structures

from gui import Visualizer, Point2D, Line2D, Box2D, add_line

root = tk.Tk()
vis = Visualizer(root,800,600)

b = structures.Box(np.array([300,200]),np.array([200,200]))
env = structures.Environment()
env.add_polygon(b)

path = structures.Path()
path.generate_path()
path.perturb_path()
path.generate_path_normals()

def draw():
    vis.clear()
    def draw_path(states):
        prev = None
        for state in states:
            d = state.penetration_depth(b)
            radius = 3
            if d:
                radius += d/10.
            if b.point_in(state.location):
                vis.add_drawable(Point2D(state.location,fill="blue",radius=radius))
            else:
                vis.add_drawable(Point2D(state.location,fill="red",radius=radius))
            if prev is not None:
                vis.add_drawable(Line2D(prev,state.location))
            prev = state.location

    for index,normal_set in enumerate(path.normals):
        if normal_set is not None:
            for normal in normal_set:
                vis.add_drawable(Line2D(path.points[index],normal*50+path.points[index]))
                vis.add_drawable(Point2D(normal*50+path.points[index],fill="green"))

    draw_path(path.states)
    for edge in b.edges:
        vis.add_drawable(Line2D(edge.start,edge.end))

draw()
vis.run()

for i in range(10):
    print "Press enter to continue",
    print "Path cost: ",path.cost(env)
    raw_input()
    path.update(env)
    draw()
    vis.draw()

root.mainloop()
