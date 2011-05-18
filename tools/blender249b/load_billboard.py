#!BPY

"""
Name: 'load_billboard'
Blender: 249
Group: 'Import'
Tooltip: 'Import Billboard cloud'
"""

import bpy
import Blender
from Blender import *
from Blender.Mathutils import *
from collections import deque

PARAMS = {
	'MakeTransp': 1, 										        # Make face transparent in viewport
	'MatProps': {'Col': [1.0, 0.0, 0.0], 'Shadeless': 0, 'Ref': 0.8, 'Spec': 0.2, 'Hard': 200, 'Alpha': 0.0, 'ZTransp': 1},
	'TexProps': {'UseAlpha': 1, 'CalcAlpha': 0, 'ExtendMode': 0},   # Texture Properties
	'TexChannel': 0, 										        # Texture Channel, start from 0
	'TexMapTo': {'Col': 1, 'Alpha': 1},                             # Map to Col and/or Alpha
}

def parse_file(file):
    '''
    Parse the Billboard.scheme, and return the vertex array, texture array and texture's path
    '''
    verts = []
    texs = []
    tex_path = ""

    v_cnt = int(file.readline().rstrip())

    #for each vertex
    for v in xrange(v_cnt):
        vec = [float(x) for x in file.readline().rstrip().split(' ')]
        verts.append(Vector(vec[0], vec[1], vec[2]))

    #for each tex coords
    for t in xrange(v_cnt):
        vec = [float(x) for x in file.readline().rstrip().split(' ')]
        texs.append(Vector(vec[0], vec[1]))

    tex_path = file.readline().rstrip()

    return verts, texs, tex_path

def re_order(v):
    '''
    Re-order the a,b,e,f sequence to a,e,f,b
    '''
    if len(v) == 4:
        return [v[0], v[2], v[3], v[1]]

def construct_frame(name, vert4, tex4, img):
    '''
    Construct a single frame mesh(called 'name') from the given 4 vertices, 4 texture coordinates and Image.
    Each frame has separate material and texture.
    '''
    # construct the mesh
    mesh = Mesh.New(name)
    mesh.verts.extend(re_order(vert4))
    mesh.faces.extend([0, 1, 2, 3])
    mesh.faces[0].image = img
    mesh.faces[0].uv = re_order(tex4)
    mesh.faces[0].transp = Mesh.FaceTranspModes.ALPHA

	# create material
    mat = Material.New(name)
    properties = PARAMS['MatProps']
    mat.setRGBCol(properties['Col'])
    mat.setRef(properties['Ref'])
    mat.setSpec(properties['Spec'])
    mat.setHardness(properties['Hard'])
    mat.setAlpha(properties['Alpha'])
    
    if properties['Shadeless']:
        mat.mode |= Material.Modes.SHADELESS
    if properties['ZTransp']:
        mat.mode |= Material.Modes.ZTRANSP
	
	properties = PARAMS['TexProps']
	
    # assign texture to material
	tex = Texture.New(name)
	tex.setType('Image')
	tex.setImage(img)
	if properties['UseAlpha']:
		tex.useAlpha = Texture.ImageFlags.USEALPHA
        tex.interpol = 0 #Texture.ImageFlags.INTERPOL
        tex.mipmap = 0 #Texture.ImageFlags.MIPMAP
			
	if properties['CalcAlpha']:
		tex.calcAlpha = Texture.ImageFlags.CALCALPHA
		
	if properties['ExtendMode']:
		tex.setExtend('Extend')
		
	texMapSetters = Texture.TexCo.UV
	
	# PARAMS['TexMapTo']['Col'] (and alpha) will either be 0 or 1 because its from a toggle, otherwise this line doesn't work
	texChanSetters = Texture.MapTo.COL * PARAMS['TexMapTo']['Col'] | Texture.MapTo.ALPHA * PARAMS['TexMapTo']['Alpha']
	
	mat.setTexture(PARAMS['TexChannel'], tex, texMapSetters, texChanSetters)
	mesh.materials += [mat]
	
    return mesh

def construct_foilage(name, verts, texs, img):
    '''
    Construct all the frames as a mesh object and apply one material and texture to it
    '''
    # reorder verts and texs
    face_list = []
    for i in xrange(len(verts)/4):
        verts[i*4:i*4+4] = re_order(verts[i*4:i*4+4])
        texs[i*4:i*4+4] = re_order(texs[i*4:i*4+4])
        face_list.append(range(i*4,i*4+4))

    # construct mesh
    mesh = Mesh.New(name+'Mesh')
    mesh.verts.extend(verts)
    mesh.faces.extend(face_list)

    for i, f in enumerate(mesh.faces):
        f.image = img
        f.uv = texs[i*4:i*4+4]
        f.transp = Mesh.FaceTranspModes.ALPHA

	# create material
    mat = Material.New(name+'Mat')
    properties = PARAMS['MatProps']
    mat.setRGBCol(properties['Col'])
    mat.setRef(properties['Ref'])
    mat.setSpec(properties['Spec'])
    mat.setHardness(properties['Hard'])
    mat.setAlpha(properties['Alpha'])
    
    if properties['Shadeless']:
        mat.mode |= Material.Modes.SHADELESS
    if properties['ZTransp']:
        mat.mode |= Material.Modes.ZTRANSP
	
	properties = PARAMS['TexProps']
	
    # assign texture to material
	tex = Texture.New(name+'Tex')
	tex.setType('Image')
	tex.setImage(img)
	if properties['UseAlpha']:
		tex.useAlpha = Texture.ImageFlags.USEALPHA
        tex.interpol = 0 #Texture.ImageFlags.INTERPOL
        tex.mipmap = 0 #Texture.ImageFlags.MIPMAP
			
	if properties['CalcAlpha']:
		tex.calcAlpha = Texture.ImageFlags.CALCALPHA
		
	if properties['ExtendMode']:
		tex.setExtend('Extend')
		
	texMapSetters = Texture.TexCo.UV
	
	# PARAMS['TexMapTo']['Col'] (and alpha) will either be 0 or 1 because its from a toggle, otherwise this line doesn't work
	texChanSetters = Texture.MapTo.COL * PARAMS['TexMapTo']['Col'] | Texture.MapTo.ALPHA * PARAMS['TexMapTo']['Alpha']
	
	mat.setTexture(PARAMS['TexChannel'], tex, texMapSetters, texChanSetters)
	mesh.materials += [mat]

    return mesh

def import_billboard(path):
    '''
    Parse the billboard(.bb) file and construct the rectangular frame one by one
    '''
    # parse file and get the vertex array, texture array and texture's path
    input = open(path, 'r')
    verts, texs, tex_path = parse_file(input)
    input.close()

	# load the tiled texture
    img = Image.Load(tex_path)

    # add frames to current scene
    scn = Scene.GetCurrent()

    # 4 verts and 4 tex per frame
    if False:
        for i in xrange(len(verts)/4):
            name = 'frame_%d' % i
            frame = construct_frame(name, verts[i*4:i*4+4], texs[i*4:i*4+4], img)
            ob = scn.objects.new(frame, name)

    if True:
        name = Blender.sys.makename(path, strip = 1)
        foilage = construct_foilage(name, verts, texs, img)
        ob = scn.objects.new(foilage, name)

    Blender.Redraw()

def main():
    Blender.Window.FileSelector(import_billboard, 'Import')

if __name__ == "__main__":
    main()
