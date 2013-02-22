import numpy as np
from itertools import izip_longest


def orient_2d(p,q,r):
    """
        > 0 if CCW
        < 0 if CW
        = 0 if colinear
    """
    return (q[0]-p[0])*(r[1]-p[1]) - (r[0]-p[0])*(q[1]-p[1])

def linspace2d(start,end,num):
	return zip(np.linspace(start[0],end[0],num),np.linspace(start[1],end[1],num))

def intersects(seg1,seg2):
    return \
        orient_2d(seg2.start,seg2.end,seg1.start)* \
        orient_2d(seg2.start,seg2.end,seg1.end)<=0 \
        and \
        orient_2d(seg1.start,seg1.end,seg2.start)* \
        orient_2d(seg1.start,seg1.end,seg2.end)<=0


"""
    From http://stackoverflow.com/questions/3252194/numpy-and-line-intersections
"""
def perp(a) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b
def intersection_point(seg1, seg2):
    if not intersects(seg1,seg2):
        return None
    a1 = seg1[0]
    a2 = seg1[1]
    b1 = seg2[0]
    b2 = seg2[1]
    
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot( dap, db)
    num = np.dot( dap, dp )
    return (num / denom)*db + b1

def grouper(n, iterable, fillvalue=None):
    "grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
    args = [iter(iterable)] * n
    return izip_longest(fillvalue=fillvalue, *args)
