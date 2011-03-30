#!BPY

"""
Name: 'bdlsg2arm'
Blender: 249
Group: 'Import'
Tooltip: 'Import BDLSkeletonGraph  to armature'
"""

import bpy
import Blender
from Blender import *
from Blender.Mathutils import *
from collections import deque

debug_out = open('/tmp/blender_log', 'a')

def parse_file(file):
    v_cnt = int(file.readline().rstrip())
    verts = []

    for i in xrange(v_cnt):
        vec = [float(x) for x in file.readline().rstrip().split(' ')]
        verts.append(Vector(vec[0], vec[1], vec[2]))
        
    l_cnt = int(file.readline().rstrip())
    links = []

    for i in xrange(l_cnt):
        l = [int(x) for x in file.readline().rstrip().split(' ')]
        links.append((l[0], l[1]))

    return (verts, links)

def construct_arm(graph):
    if not graph:
        return

    verts = graph[0]
    links = graph[1]

    bone_id = {} #bone_id[1] is the root bone

    #for each link, new a bone, and put it in bone_id
    for l in links:
        eb = Armature.Editbone()
        eb.head = verts[l[0]]
        eb.tail = verts[l[1]]
        bone_id[l[1]] = eb

    #setup an armature object for holding all bones
    arm = Blender.Armature.Armature('Armature')
    arm.drawType = Armature.STICK
    scene = Scene.GetCurrent()
    scene.objects.new(arm, 'Armature')
    arm.makeEditable()
    for l in links:
        arm.bones['bone.%03d' % l[1]] = bone_id[l[1]]

    #for each link (or bone), attach itself to its parent
    for l in links:
        if l[0] != 0:
            bone = bone_id[l[1]]
            bone.parent = bone_id[l[0]]
            bone.options = [Armature.CONNECTED]

    #exit EditMode
    arm.update()

def import_bdlsg(path):
    input = open(path, 'r')
    graph = parse_file(input)
    construct_arm(graph)
    Blender.Redraw()
    input.close()

    debug_out.close()

def main():
    #dump_bones()
    Blender.Window.FileSelector(import_bdlsg, 'Import')

if __name__ == "__main__":
    main()