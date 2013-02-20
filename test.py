import Tkinter as tk
import numpy as np
from path import utils

from gui import Visualizer, Point2D, Line2D, Box2D, add_line

root = tk.Tk()
vis = Visualizer(root,800,600)

b = utils.Box(np.array([300,200]),np.array([200,200]))
path = utils.Path()
path.generate_path()
path.perturb_path()
path.get_path_normals()
def draw_path(points):
	prev = None
	for n in points:
		if b.point_in(n):
			vis.add_drawable(Point2D(n,fill="blue"))
		else:
			vis.add_drawable(Point2D(n,fill="red"))
		if prev is not None:
			vis.add_drawable(Line2D(prev,n))
		prev = n

for index,normal_set in enumerate(path.normals):
	if normal_set is not None:
		for normal in normal_set:
			vis.add_drawable(Line2D(path.points[index],normal*50+path.points[index]))
			vis.add_drawable(Point2D(normal*50+path.points[index],fill="green"))

draw_path(path.points)
vis.add_drawable(Box2D(np.array([300,200]),np.array([200,200])))

vis.run()
root.mainloop()
