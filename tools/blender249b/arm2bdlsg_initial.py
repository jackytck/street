#!BPY

"""
Name: 'arm2bdlsg'
Blender: 249
Group: 'Export'
Tooltip: 'Export armature to initial BDLSkeleton *'
"""

import bpy
import Blender
from Blender import *
from collections import deque

debug_out = open('/tmp/blender_log', 'a')

def dump_bones():
    bones = Armature.Get().get('Armature')

    if not bones:
        return
    print >> debug_out, ''

    bones = bones.bones.values()

    for b in bones:
        head = b.head['ARMATURESPACE']
        tail = b.tail['ARMATURESPACE']
        print >> debug_out, 'head (%f %f %f)\ttail (%f %f %f)' % (head.x, head.y, head.z, tail.x, tail.y, tail.z)

def bfs(root, output):
    if not root:
        return

    v_cnt = 0
    l_cnt = 0
    s_cnt = 0
    vertices = ""
    links = ""
    supports = ""

    bone_id = {}
    bone2v_id = {}

    queue = deque([root])

    #bfs, setup vertices, bone_id, bone2v_id
    while queue:
        front = queue.popleft()#a bone object
        head = front.head.get('ARMATURESPACE') if not front.hasParent() else None
        tail = front.tail.get('ARMATURESPACE')

        bone_id[front.name] = l_cnt

        if head:
            vertices += '%f %f %f\n' % (head.x, head.y, head.z)
            v_cnt += 1

        if tail:
            vertices += '%f %f %f\n' % (tail.x, tail.y, tail.z)
            bone2v_id[l_cnt] = v_cnt
            v_cnt += 1
            l_cnt += 1

        for child in front.children:
            queue.append(child)

    #bfs, setup links
    queue = deque([root])
    while queue:
        front = queue.popleft()#a bone, also a link
        tail_id = bone2v_id[bone_id[front.name]]

        if not front.hasParent():
            links += '0 %d\n' % tail_id

        for child in front.children:
            links += '%d %d\n' % (tail_id, bone2v_id[bone_id[child.name]])
            queue.append(child)

    #bfs, setup supporting links, for initial skeleton,
    #assume leaf link is a supporting link
    queue = deque([root])
    while queue:
        front = queue.popleft()

        if not front.children:
            supports += '%d %d\n' % (bone2v_id[bone_id[front.parent.name]], bone2v_id[bone_id[front.name]])
            s_cnt += 1

        for child in front.children:
            queue.append(child)

    print >> output, v_cnt
    print >> output, vertices,
    print >> output, l_cnt
    print >> output, links,
    print >> output, s_cnt
    print >> output, supports,
    print >> output, 0

def export(path):
    name = Blender.Object.GetSelected()[0].getData(name_only=True)
    bones = Armature.Get().get(name)

    if not bones:
        return
    print >> debug_out, ''

    output = open(path, 'w')
    bones = bones.bones.values()

    #find a list of tree roots
    roots = []
    for b in bones:
        if not b.hasParent():
            roots.append(b)
            print >> debug_out, b.name

    #bfs each root
    for b in roots:
        bfs(b, output)
        if b != roots[-1]:
            print >> output, ''

    output.close()
    debug_out.close()

def main():
    #dump_bones()
    Blender.Window.FileSelector(export, "Export")

if __name__ == "__main__":
    main()
