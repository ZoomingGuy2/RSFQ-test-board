#!/usr/bin/env python
# modified viafence.py from the RF-Tools via fence generator plugin
# modded version capable of adding via fences continuing past length tuning pattern

import math
from bisect import bisect_left
import wx
import pcbnew

###############################################################################
# DEBUG UTILITIES
###############################################################################

DEBUG = False

def dbg(msg):
    if DEBUG:
        try:
            wx.LogMessage("[viafence] " + str(msg))
        except:
            print("[viafence]", msg)

###############################################################################
# BASIC GEOMETRY
###############################################################################

def getLineSlope(line):
    return math.atan2(line[0][1]-line[1][1], line[0][0]-line[1][0])

def getLineLength(line):
    return math.hypot(line[0][0]-line[1][0], line[0][1]-line[1][1])

def getPathCumDist(path):
    cum = [0.0]
    for i in range(1, len(path)):
        cum.append(cum[-1] + getLineLength([path[i], path[i-1]]))
    return cum

###############################################################################
# INTERPOLATION
###############################################################################

class LinearInterpolator:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.m = [(y[i+1]-y[i])/(x[i+1]-x[i]) for i in range(len(x)-1)]

    def __call__(self, xq):
        i = max(0, bisect_left(self.x, xq)-1)
        return self.y[i] + self.m[i]*(xq-self.x[i])

class PathInterpolator:
    def __init__(self, t, path):
        self.xi = LinearInterpolator(t, [p[0] for p in path])
        self.yi = LinearInterpolator(t, [p[1] for p in path])

    def __call__(self, t):
        return [self.xi(t), self.yi(t)]

###############################################################################
# FIXED DISTRIBUTOR (GLOBAL SPACING, NO RESET)
###############################################################################

def distributeAlongPathWithCarry(path, pitch, carry):
    """
    Correct via distributor:
    - carries spacing across fragmented paths
    - never silently drops vias
    """
    if len(path) < 2:
        return [], carry

    cum = getPathCumDist(path)
    total = cum[-1]

    if total <= 0:
        return [], carry

    interp = PathInterpolator(cum, path)
    vias = []

    dist = carry
    while dist + pitch <= total:
        dist += pitch
        vias.append(interp(dist))

    new_carry = dist - total
    dbg(f"segment len={total/1e6:.3f}mm, vias={len(vias)}, carry={new_carry/1e6:.3f}mm")
    return vias, new_carry

###############################################################################
# POLYGON OPS (UNCHANGED)
###############################################################################

def expandPathsToPolygons(pathList, offset):
    import pyclipper
    co = pyclipper.PyclipperOffset()
    for p in pathList:
        co.AddPath(p, pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
    return co.Execute(offset)

def isPointInPolygon(pt, poly):
    import pyclipper
    return pyclipper.PointInPolygon(pt, poly) == 1

def getPathsInsidePolygon(pathList, poly):
    out = []
    for p in pathList:
        for v in p:
            if isPointInPolygon(v, poly):
                out.append(p)
                break
    return out

###############################################################################
# MAIN ENTRY POINT
###############################################################################

def generateViaFence(pathList, viaOffset, viaPitch, vFunc=lambda *a,**k:None):
    global DEBUG
    DEBUG = vFunc is not None

    dbg(f"input paths: {len(pathList)}")

    # Remove degenerate paths
    pathList = [p for p in pathList if getLineLength(p) > 0]

    polygons = expandPathsToPolygons(pathList, viaOffset)
    dbg(f"generated {len(polygons)} offset polygons")

    viaPoints = []
    processed = set()
    carry = 0.0

    # Largest polygons first (important for tuning)
    def poly_area(p):
        xs = [x for x,_ in p]
        ys = [y for _,y in p]
        return (max(xs)-min(xs))*(max(ys)-min(ys))

    polygons = sorted(polygons, key=poly_area, reverse=True)

    for poly in polygons:
        localPaths = getPathsInsidePolygon(pathList, poly)

        newPaths = []
        for p in localPaths:
            if id(p) not in processed:
                processed.add(id(p))
                newPaths.append(p)

        if not newPaths:
            continue

        dbg(f"processing {len(newPaths)} paths in polygon")

        for path in newPaths:
            left  = offsetPath(path,  viaOffset)
            right = offsetPath(path, -viaOffset)
        
            ptsL, carry = distributeAlongPathWithCarry(left,  viaPitch, carry)
            ptsR, carry = distributeAlongPathWithCarry(right, viaPitch, carry)
        
            viaPoints.extend(ptsL)
            viaPoints.extend(ptsR)


        dbg(f"total vias generated: {len(viaPoints)}")
    return viaPoints

###############################################################################
# ARC SUPPORT (REQUIRED BY viafence_action.py)
###############################################################################

def getCircleCenterRadius(sp, ep, ip):
    """
    Returns center point and radius of a circle defined by three points:
    start, end, intermediate (arc mid)
    """
    x1, y1 = float(sp.x), float(sp.y)
    x2, y2 = float(ep.x), float(ep.y)
    x3, y3 = float(ip.x), float(ip.y)

    x12 = x1 - x2
    x13 = x1 - x3
    y12 = y1 - y2
    y13 = y1 - y3
    y31 = y3 - y1
    y21 = y2 - y1
    x31 = x3 - x1
    x21 = x2 - x1

    sx13 = x1*x1 - x3*x3
    sy13 = y1*y1 - y3*y3
    sx21 = x2*x2 - x1*x1
    sy21 = y2*y2 - y1*y1

    f = (sx13*x12 + sy13*x12 + sx21*x13 + sy21*x13) / (2*(y31*x12 - y21*x13))
    g = (sx13*y12 + sy13*y12 + sx21*y13 + sy21*y13) / (2*(x31*y12 - x21*y13))

    cx = -g
    cy = -f
    r = math.sqrt(cx*cx + cy*cy - (-x1*x1 - y1*y1 - 2*g*x1 - 2*f*y1))

    return wx.Point(int(cx), int(cy)), r


def getAngleRadians(p1, p2):
    return math.atan2((p1.y - p2.y), (p1.x - p2.x))


def rotatePoint(r, sa, da, c):
    """
    Rotate a point around center c by angle da (radians)
    """
    x = c.x - math.cos(sa + da) * r
    y = c.y - math.sin(sa + da) * r
    return wx.Point(int(x), int(y))

###############################################################################
# ARC DISCRETIZATION (REQUIRED BY viafence_action.py)
###############################################################################

def create_round_pts(sp, ep, cntr, rad, layer, width, netName, N_SEGMENTS):
    """
    Discretize a KiCad arc into line segments.
    Returns a list of wx.Point suitable for via fence generation.
    """
    start_point = sp
    end_point = ep

    a1 = getAngleRadians(cntr, sp)
    a2 = getAngleRadians(cntr, ep)

    # Determine shortest rotation direction
    if (a2 - a1) > 0 and abs(a2 - a1) > math.radians(180):
        deltaA = -(math.radians(360) - (a2 - a1)) / N_SEGMENTS
    elif (a2 - a1) < 0 and abs(a2 - a1) > math.radians(180):
        deltaA = (math.radians(360) - abs(a2 - a1)) / N_SEGMENTS
    else:
        deltaA = (a2 - a1) / N_SEGMENTS

    points = []
    delta = 0.0

    for _ in range(N_SEGMENTS + 1):
        x = cntr.x - math.cos(a1 + delta) * rad
        y = cntr.y - math.sin(a1 + delta) * rad
        points.append(wx.Point(int(x), int(y)))
        delta += deltaA

    return points

def offsetPath(path, offset):
    """
    Generate a parallel offset of a polyline.
    Works well for PCB tracks (piecewise linear).
    """
    out = []
    for i in range(len(path)):
        if i == 0:
            p0, p1 = path[0], path[1]
        else:
            p0, p1 = path[i-1], path[i]

        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        L = math.hypot(dx, dy)
        if L == 0:
            continue

        nx = -dy / L
        ny =  dx / L

        out.append([
            int(path[i][0] + offset * nx),
            int(path[i][1] + offset * ny)
        ])
    return out



