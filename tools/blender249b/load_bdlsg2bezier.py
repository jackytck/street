#!BPY

"""
Name: 'load_bdlsg2bezier'
Blender: 249
Group: 'Import'
Tooltip: 'Import BDLSG2Bezier to BezierCurves'
"""

import bpy
import Blender
from Blender import *
from Blender.Mathutils import *
from collections import deque

def add_branch_to_curve(curve, bezier_triples, radii):
    '''
    Add a list of BezTriple to the curve
    '''
    if not bezier_triples or not radii:
        return

    curve.appendNurb(bezier_triples[0]) # We must add with a point to start with
    cu_nurb = curve[-1]

    i = 1 # skip first bezier triple because it was used to init the curve
    while i<len(bezier_triples):
        cu_nurb.append(bezier_triples[i])
        i += 1

def bezFromVecs(vecs, r):
    '''
    Bezier triple from 3 vecs, shortcut functon
    '''
    bt = BezTriple.New(vecs[0].x, vecs[0].y, vecs[0].z, vecs[1].x, vecs[1].y, vecs[1].z, vecs[2].x, vecs[2].y, vecs[2].z)
    bt.handleTypes = (BezTriple.HandleTypes.FREE, BezTriple.HandleTypes.FREE)
    bt.radius = r
    return bt

def smooth_radius(radii):

    rlen = len(radii)
    if(rlen < 2):
        return None
    start = radii[0]
    end = radii[-1]

    i = 1
    while i<rlen:
        fac = float(1+i) / rlen
        radii[i] = start*(1.0-fac) + end*fac
        i += 1

    return radii

def parse_file(file, curve):
    '''
    Parse the BDLSG2Bezier.scheme, and construct the curve on the way
    '''
    b_cnt = int(file.readline().rstrip())

    #for each branch
    for b in xrange(b_cnt):

        bezier_triples = []
        vecs3_list = []
        radii = []
        v_cnt = int(file.readline().rstrip())

        for i in xrange(v_cnt):
            vec = [float(x) for x in file.readline().rstrip().split(' ')]
            vecs3 = [Vector(vec[0], vec[1], vec[2]), Vector(vec[3], vec[4], vec[5]), Vector(vec[6], vec[7], vec[8])]
            vecs3_list.append(vecs3)

        for i in xrange(v_cnt):
            r = float(file.readline().rstrip())
            radii.append(r)

        radii = smooth_radius(radii)

        for i in xrange(v_cnt):
            bt = bezFromVecs(vecs3_list[i], radii[i])
            bezier_triples.append(bt)

        #construct bezier curve
        add_branch_to_curve(curve, bezier_triples, radii)

def import_bdlsg2bezier(path):
    '''
    Parse the bdlsg2bezier file and construct the corresponding Bezier curve
    '''
    curve = Curve.New()
    curve.setFlag(1) # set curve to 3d
    curve.setBevresol(6) # set the Curve's bevel resolution value
    curve.setExt2(0.1) # set the Curve's ext2 value

    #parse file and append CurNurb to curve for each branch on the fly
    input = open(path, 'r')
    parse_file(input, curve)
    input.close()

    #add curve to scene
    scn = Scene.GetCurrent()
    ob = scn.objects.new(curve)
    Blender.Redraw()

def main():
    Blender.Window.FileSelector(import_bdlsg2bezier, 'Import')

if __name__ == "__main__":
    main()
