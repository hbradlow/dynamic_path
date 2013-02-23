import Tkinter as tk
import numpy as np
from path import utils
from path import structures

from gui import Visualizer, Point2D, Line2D, Box2D, add_line

root = tk.Tk()

def key_callback(event):
    def draw_callback():
        draw()
        vis.draw()
        root.update_idletasks()
    path.benchmark(env,draw_callback=draw_callback,iterations=10)
    #path.update(env)

vis = Visualizer(root,800,600,key_callback=key_callback)

b = structures.Box(np.array([250,150]),np.array([150,200]))
b2 = structures.Box(np.array([450,250]),np.array([150,200]))
env = structures.Environment()
env.add_polygon(b)
env.add_polygon(b2)

path = structures.Path()
path.generate_path()
path.perturb_path()
path.generate_path_normals()

def draw():
    vis.clear()
    def draw_path(states):
        prev = None
        for state in states:
            radius = 3
            if env.point_in(state.location):
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
    for edge in b2.edges:
        vis.add_drawable(Line2D(edge.start,edge.end))

draw()
vis.run()
root.mainloop()
