#!BPY

"""
Name: 'loadcache'
Blender: 249
Group: 'Import'
Tooltip: 'Import cache for visualization and debugging'
"""

import bpy
import Blender
from Blender import *
from Blender.Mathutils import *
from collections import deque

def parse_file(file):
	k_d = float(file.readline().rstrip())
	cnt = int(file.readline().rstrip())
	pts = []

	for i in xrange(cnt):
		p = [float(x) for x in file.readline().rstrip().split(' ')]
		pts.append(p)

	return pts

def add_cube(x, y, z, name, passedMesh):
	ob = Object.New("Mesh",name)
	ob.LocX = x
	ob.LocY = y
	ob.LocZ = z

	ob.link(passedMesh)
	return ob

def visualize(pts):
	cube_mesh = Mesh.Primitives.Cube(0.3)
	scene = Scene.GetCurrent()

	group = Group.New('cache_cube_group')
	cubes = []
	for i, p in enumerate(pts):
		cubes.append(add_cube(p[0], p[1], p[2], 'cube_%d' % i, cube_mesh))
	group.objects = cubes

	grp_obj = scene.objects.new('Empty', 'cache_group')
	grp_obj.enableDupGroup = True
	grp_obj.DupGroup = group
        
def import_cache(path):
	input = open(path, 'r')
	points = parse_file(input)
	visualize(points)
	Blender.Redraw()
	input.close()

def main():
	Blender.Window.FileSelector(import_cache, 'Import')

if __name__ == "__main__":
	main()
