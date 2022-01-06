
bl_info = {
	"name": "Export pbrt3",
	"author": "Jonathan Klein",
	#"version": (0, 0, 1),
	"blender": (2, 78, 0),
	#"location": "File > Import-Export",
	"description": "Exports the scene as an pbrt3 input file",
	"support": 'COMMUNITY',
	"category": "Import-Export"
}


# path where the pbrt3 executable can be found
pbrt3Path = None
#pbrt3Path = r"C:\pbrt-v3\bin\TransientRenderer.exe"

if not pbrt3Path:
	raise Exception("Please specify the path to the local pbrt executable above")


# path where temporary files during rendering (.pbrt scene file, .pfm image file) will be stored
tmpPath = r"C:\tmp\\"


import bpy
from math import pi as pi
from mathutils import Matrix, Vector
import datetime # for export timestamp




def GenerateSceneWideRenderingOptionsString(context, filename : str) -> str:
	s = ""
	
	### Camera & Film
	cameras = [c for c in context.scene.objects if c.type=='CAMERA']

	if len(cameras) != 1:
			raise Exception("You need exactly one camera!")

	co = cameras[0]
	cc = co.data
	pos = co.location

	transMat = Matrix.Translation(pos)
	co.rotation_mode = 'QUATERNION'
	rotMat = co.rotation_quaternion.to_matrix().to_4x4()
	transf = transMat*rotMat
	look = transf * Vector((0, 0, -1, 1))
	up = transf * Vector((0, 1, 0, 0))

	s += "Scale -1 1 1\n\n"
	s += "LookAt\n	{} {} {}\n	{} {} {}\n	{} {} {}\n\n".format(
		pos.x,
		pos.y,
		pos.z,
		look.x,
		look.y,
		look.z,
		up.x,
		up.y,
		up.z)

	# the FOV can either be relative to the width or the height. Due to different conventions we have to account for that
	# by setting the screenWindow. 
	render = context.scene.render
	ratio = render.resolution_y / render.resolution_x
	w = 1
	h = 1
	if ratio <= 1: #screenWindow values apparently should be smaller 1
		h = ratio
	else:
		w = 1/ratio
		
	if(cc.type == 'PERSP'):
		s += 'Camera "perspective"\n	"float fov" [{}]\n'.format(cc.angle/pi*180.0)
		s += '	"float screenwindow" [{} {} {} {}]\n\n'.format(-w, w, -h, h)
	elif(cc.type == 'ORTHO'):
		w *= cc.ortho_scale / 2
		h *= cc.ortho_scale / 2
		s += 'Camera "orthographic"\n'
		s += '	"float screenwindow" [{} {} {} {}]\n\n'.format(-w, w, -h, h)
	else:
		raise Exception("camera must be perspective!")

	
	s += 'Film "image"\n	"string filename" ["{}"]\n	"integer xresolution" [{}]\n	"integer yresolution" [{}]\n'.format(
		filename.replace("\\", "/"), # it seems, that the pbrt parser also has some escaping. we avoid this by exchanging the directory separators
		render.resolution_x*render.resolution_percentage/100.0,
		render.resolution_y*render.resolution_percentage/100.0)
	
	# we have some additional Film options for TransientFilm
	if "transientpath" == context.scene.pbrt3_integrator:
		s+= '	"integer tresolution" [{}]\n	"float t_min" [{}]\n	"float t_max" [{}]\n'.format(
			context.scene.pbrt3_time_bins,
			context.scene.pbrt3_t_min,
			context.scene.pbrt3_t_max)
	
	s += '\n\n'
	
	s += 'Integrator "{}"\n'.format(context.scene.pbrt3_integrator)
	if "transientpath" == context.scene.pbrt3_integrator:
		ignore = '"true"' if context.scene.pbrt3_ignore_distance_to_camera else '"false"'
		s+= '	"bool ignoreDistanceToCamera" [{}]\n'.format(ignore)
	s+= '	"integer maxdepth" [{}]\n'.format(context.scene.pbrt3_max_bounces)
	s+='\n'

	# render settings:
	s += 'Sampler "lowdiscrepancy" "integer pixelsamples" {}\n\n'.format(context.scene.pbrt3_samples)

	
	# add some meta information about this file
	
	if "transientpath" == context.scene.pbrt3_integrator:
		currentTime = datetime.datetime.utcnow().replace(tzinfo=datetime.timezone.utc).isoformat()
		currentFrame = context.scene.frame_current
		filename = bpy.context.blend_data.filepath
		
		s += 'FileInformation ""\n'
		s += '	"string BlenderFilename" ["{}"]\n'.format(filename.replace("\\", "\\\\"))
		s += '	"integer BlenderCurrentFrame" [{}]\n'.format(currentFrame)
		s += '	"string ExportTime" ["{}"]\n'.format(currentTime)
		s += '\n\n'
	
	return s



def GenerateLightString(light) -> str:
	s = 'AttributeBegin\n'
	if light.data.type == 'POINT':
		pos = light.location
		color = light.data.color * light.data.energy
		s+= '	LightSource "point" "point from" [{} {} {}] "rgb I" [{} {} {}]\n'.format(
			pos.x,
			pos.y,
			pos.z,
			color.r,
			color.g,
			color.b)
	elif light.data.type == 'SUN':
		color = light.data.color * light.data.energy
		light.rotation_mode = 'QUATERNION'
		rotMat = light.rotation_quaternion.to_matrix().to_4x4()
		dir = rotMat * Vector((0, 0, -1, 0))
		# point from remains on (0, 0, 0)
		s+= '	LightSource "distant" "point to" [{} {} {}] "rgb I" [{} {} {}]\n'.format(
			dir.x,
			dir.y,
			dir.z,
			color.r,
			color.g,
			color.b)
	else:
		raise Exception("Light source ", light.data.type, " is currently unsupported")
		
	s += 'AttributeEnd\n\n'
	return s



def GenerateMeshString(obj, scene) -> str:
	# Meshes
	def TriangulateMesh(me):
		import bmesh
		bm = bmesh.new()
		bm.from_mesh(me)
		bmesh.ops.triangulate(bm, faces=bm.faces)
		bm.to_mesh(me)
		bm.free()
		
	s = ""
	
	# --- prefetch data ---
	
	# create a new mesh with all modifiers applied
	mesh = obj.to_mesh(scene, True, 'RENDER', calc_tessface=False)
	
	TriangulateMesh(mesh)
	indices = list()
	for poly in mesh.polygons:
		if not poly.use_smooth:
			raise Exception(obj.name+": hard shading currently not supported - use edge split modifier!") 
		for i in poly.loop_indices:
			indices.append(mesh.loops[i].vertex_index)
			
	positions = list()
	for v in mesh.vertices:
		positions.extend((v.co.x, v.co.y, v.co.z))
	
	normals = list()
	for n in mesh.vertices:
		normals.extend((n.normal.x, n.normal.y, n.normal.z))
	
	# remove the temporary mesh again
	# (would happen anyway if you reloaded the scene in blender, but we want to be nice)
	bpy.data.meshes.remove(mesh)
	
	# --- write data ---
	s += '# {} - {}\n'.format(obj.name, obj.data.name)
	s += 'AttributeBegin\n'
	
	# the transformation
	obj.rotation_mode = 'QUATERNION'
	loc = obj.matrix_world.decompose()[0]
	rot = obj.matrix_world.decompose()[1]
	sca = obj.matrix_world.decompose()[2]
	
	s += '	Translate {} {} {}\n'.format(loc.x, loc.y, loc.z)
	
	axis, angle = rot.to_axis_angle()
	s += '	Rotate {} {} {} {}\n'.format(angle/pi*180, axis.x, axis.y, axis.z)
	
	s += '	Scale {} {} {}\n'.format(sca.x, sca.y, sca.z)
	
	
	# --- material ---
	material = obj.active_material	
	if not hasattr(material, 'pbrt3_type'):
		raise Exception(obj.name+" has no pbrt material")
	t = material.pbrt3_type
	if 'matte' == t:
		c = material.pbrt3_matte_kd * material.pbrt3_matte_kdscale
		s += '	Material "matte" "rgb Kd" [ {} {} {} ] "float sigma" [{}]\n'.format(
			c.r, c.g, c.b, material.pbrt3_matte_sigma)
	elif 'metal' == t:
		r = material.pbrt3_metal_roughness
		eta = material.pbrt3_metal_eta
		k = material.pbrt3_metal_k
		s += '	Material "metal" "float roughness" {} "rgb eta" [{} {} {}] "rgb k" [{} {} {}]\n'.format(
			r, eta, eta, eta, k, k, k)
	elif 'mirror' == t:
		c = material.pbrt3_mirror_kr
		s += '	Material "mirror" "rgb Kr" [ {} {} {} ]\n'.format(
			c.r, c.g, c.b)
	elif 'glass' == t:
		kr = material.pbrt3_glass_kr * material.pbrt3_glass_krscale
		kt = material.pbrt3_glass_kt * material.pbrt3_glass_ktscale
		s += '	Material "glass" "rgb Kr" [ {} {} {} ] "rgb Kt" [ {} {} {} ] "float index" [{}]\n'.format(
			kr.r, kr.g, kr.b, kt.r, kt.g, kt.b, material.pbrt3_glass_index)
	else:
		raise Exception("unsupported material type!")
		
	# --- light ---
	if material.pbrt3_light_islight:
		l = material.pbrt3_light_l * material.pbrt3_light_lscale
		s += '	AreaLightSource "diffuse" "rgb L" [ {} {} {} ]\n'.format(l.r, l.g, l.b)
	
	
	# mesh data
	s += '	Shape "trianglemesh"  # triangles: {}, positions: {}, normals: {}\n'.format(len(indices)//3, len(positions)//3, len(normals)//3)
	
	s += '		"integer indices" [' # indices
	for i in indices:
		s += str(i) + " "
	s += ']\n'
	
	s += '		"point P" [' # positions
	for p in positions:
		s += str(p) + " "
	s += ']\n'
	
	s += '		"normal N" [' # normals
	for n in normals:
		s += str(n) + " "
	s += ']\n'
	
	if int(obj.pbrt3_semantic) != 0: # ObjectSemantic
		s += '		"integer ObjectSemantic" ' + obj.pbrt3_semantic + '\n'
	
	s += 'AttributeEnd\n\n'
	
	return s


# checks if the object is in any layer that is visible
def ObjectInVisibleLayer(context, layers) -> bool:
	visible = context.scene.layers
	for i in range(len(layers)):
		if visible[i] and layers[i]:
			return True
	return False


# checks, whether this object should be exported - this will later be customized by export options
def ShouldExportObject(obj, context) -> bool:
	if ObjectInVisibleLayer(context, obj.layers) and obj.is_visible(context.scene):
		return True
	else:
		return False


def GenerateSceneString(context, filename) -> str:	
	scene = context.scene
	scene.update()
	
	s = "" # the string that contains the whole scene description

	s += GenerateSceneWideRenderingOptionsString(context, filename)

	### World
	s += 'WorldBegin\n\n'
	
	# Light sources
	lights = [l for l in context.scene.objects if l.type=='LAMP' and ShouldExportObject(l, context)]
	
	for light in lights:
		s += GenerateLightString(light)
	
	# local meshes Meshs
	meshes = [m for m in context.scene.objects if m.type=='MESH' and ShouldExportObject(m, context)]
	for mesh in meshes:
		s += GenerateMeshString(mesh, scene)

	# meshes that were linked as a group...
	groups = [g for g in context.scene.objects if g.type=='EMPTY' and ShouldExportObject(g, context) and g.dupli_group is not None]
	for g in groups:
		s += "# Group " + g.name + "\n"
		
		loc = g.matrix_world.decompose()[0]
		rot = g.matrix_world.decompose()[1]
		sca = g.matrix_world.decompose()[2]
			
		s += "AttributeBegin\n"
		s += '	Translate {} {} {}\n'.format(loc.x, loc.y, loc.z)
		axis, angle = rot.to_axis_angle()
		s += '	Rotate {} {} {} {}\n\n'.format(angle/pi*180, axis.x, axis.y, axis.z)
		s += '	Scale {} {} {}\n'.format(sca.x, sca.y, sca.z)
		
		meshes = [m for m in g.dupli_group.objects if m.type=='MESH']
		for mesh in meshes:
			if g.pbrt3_semantic != mesh.pbrt3_semantic:
				raise Exception("semantics differ! Group: "+g.name)
			s += GenerateMeshString(mesh, scene)

		s += "AttributeEnd\n\n"
		
	s += 'WorldEnd\n'
	
	return s


	
	
	
################################################# GUI stuff


from bpy_extras.io_utils import ExportHelper
from bpy.props import BoolProperty

class ExportPbrt3(bpy.types.Operator, ExportHelper):
	bl_label = "Export pbrt3" # operator name as used in the space menu
	bl_idname = "export_scene.pbrt3" # internal name, used for referencing
	
	filename_ext = ".pbrt"
		
	def execute(self, context):
		try:
			print("Starting pbrt3 Exporter...")
				
			extension =  ".ti" if context.scene.pbrt3_integrator == "transientpath" else ".exr"
			if self.FollowExportName:
				renderFilename = bpy.path.display_name(self.filepath)
			else:
				renderFilename = bpy.path.display_name(bpy.context.blend_data.filepath)
			
			if self.ExportSceneAnimation:
				sc = context.scene
				frame_orig = sc.frame_current
								
				for i in range(sc.frame_start, sc.frame_end+1):
					sc.frame_set(i)
					s = "-{num:04d}".format(num=i)
					extPos = self.filepath.rfind(".")
					f = open(self.filepath[:extPos]+s+self.filepath[extPos:], 'w', encoding='utf-8')
					f.write(GenerateSceneString(context, renderFilename+s+extension))
					f.close()
					
				sc.frame_set(frame_orig)
				
			else:
				f = open(self.filepath, 'w', encoding='utf-8')
				f.write(GenerateSceneString(context, renderFilename+extension))
				f.close()
		
			print("done")
			return {'FINISHED'}
		except Exception as e:
			self.report({'ERROR'}, repr(e))
			return {'CANCELLED'}
	
	
	def draw(self, context):
		layout = self.layout
		row = layout.row()
		row.prop(self, "FollowExportName")
		row = layout.row()
		row.prop(self, "ExportSceneAnimation")

	FollowExportName = BoolProperty(
		name='Follow exported Filename',
		description='Sets the export filename as render filename instead of the blend filename',
		default=True)
		
	ExportSceneAnimation = BoolProperty(
		name='Export scene animation',
		description='Writes a .pbrt file for every frame in the scene',
		default=False)
		

# action that is associated with the export menu entry
def MenuExport(self, context):
	self.layout.operator(ExportPbrt3.bl_idname, text="pbrt3")





def LoadPfm(filename : str):
	import struct

	f = open(filename, "rb")

	header = f.readline().decode('utf-8')
	if not header.startswith('PF'):
		raise Exception("only color images supported: ", header)

	resolution  = f.readline().decode('utf-8').split(' ')
	x = int(resolution[0])
	y = int(resolution[1])
	aspect = float(f.readline().decode('utf-8'))
	if aspect > 0:
		raise Exception("only little endian files are supported")

	data = struct.unpack("f"*x*y*3, f.read(x*y*4*3))
	return data
	

class Pbrt3Renderer(bpy.types.RenderEngine):
	bl_idname = "pbrt3_renderer"
	bl_label = "pbrt3 Render"
	bl_use_preview = False

	

	def render(self, scene):
		try:
			scale = scene.render.resolution_percentage / 100.0
			self.size_x = int(scene.render.resolution_x * scale)
			self.size_y = int(scene.render.resolution_y * scale)

			import subprocess, os
			if not os.path.isfile(pbrt3Path):
				self.report({'ERROR'}, "pbrt executable not found!")
				return
				
			f = open(tmpPath+"temp.pbrt", 'w', encoding='utf-8')
			f.write(GenerateSceneString(bpy.context, tmpPath+"temp.pfm"))
			f.close()
			subprocess.call([pbrt3Path, tmpPath+"temp.pbrt"], shell=True)
			
			print("Rendering done!")
			result = self.begin_result(0, 0, self.size_x, self.size_y)
			layer = result.layers[0].passes["Combined"]
			data = list(LoadPfm(tmpPath+"temp.pfm"))
			numPixels = self.size_x*self.size_y
			rgbaData = [ [*data[3*i : 3*(i+1)], 1] for i in range(numPixels)]
			layer.rect = rgbaData
			self.end_result(result)
			
		except Exception as e:
			self.report({'ERROR'}, repr(e))





class MaterialPanel(bpy.types.Panel):
	bl_idname = "PBRT3_MATERIAL_PANEL"
	bl_context = 'material'
	bl_label = "pbrt3"
	bl_space_type = 'PROPERTIES'
	bl_region_type = 'WINDOW'
	
	def draw(self, context):
		# some definitions, that will make our lives easier
		l = self.layout
		m = context.material
		if m is None:
			return
		t = context.material.pbrt3_type
		
		l.prop(context.material, "pbrt3_type")
		
		# build the GUI depending on the material type
		if('glass' == t):
			r = self.layout.row()
			r.prop(m, "pbrt3_glass_kr")
			r.prop(m, "pbrt3_glass_krscale")
			
			r = self.layout.row()
			r.prop(m, "pbrt3_glass_kt")
			r.prop(m, "pbrt3_glass_ktscale")
			
			l.prop(m, "pbrt3_glass_index")
		
		elif('metal' == t):
			r = self.layout.row()
			r.prop(m, "pbrt3_metal_roughness")
			r.prop(m, "pbrt3_metal_eta")
			r.prop(m, "pbrt3_metal_k")
			
		elif('mirror' == t):
			r = self.layout.row()
			r.prop(m, "pbrt3_mirror_kr")
			
		elif('matte' == t):
			r = self.layout.row()
			r.prop(m, "pbrt3_matte_kd")
			r.prop(m, "pbrt3_matte_kdscale")
			
			l.prop(m, "pbrt3_matte_sigma")
		
		r = self.layout.row()
		r.prop(m, "pbrt3_light_islight")
		if m.pbrt3_light_islight:
			r = self.layout.row()
			r.prop(m, "pbrt3_light_l")
			r.prop(m, "pbrt3_light_lscale")
			

def RegisterMaterialProperties():
	mat = bpy.types.Material
	from bpy.props import EnumProperty, FloatProperty, FloatVectorProperty
	
	# new material properties
	mat.pbrt3_type = EnumProperty(
		name = "Type",
		description = "The pbrt3 material class",
		items = (	('glass', 'Glass', ''),
					('mirror', 'Mirror', ''),
					('matte', 'Matte', 'The "matte" material defines an object with simple Lambertian scattering.'),
					('metal', 'Metal', '')
					#('kdsubsurface', 'Kd Subsurface', ''),
					#('measured', 'Measured', ''),
					#('mix', 'Mix', ''),
					#('plastic', 'Plastic', ''),
					#('shinymetal', 'Shiny Metal', ''),
					#('substrate', 'Substrate', ''),
					#('subsurface', 'Subsurface', ''),
					#('translucent', 'Translucent', ''),
					#('uber', 'Uber', ''),
				),
		default = 'matte'
		)
	
	
	# matte
	mat.pbrt3_matte_kd = FloatVectorProperty(
		name = "Kd",
		description = "The diffuse reflectivity of the surface",
		subtype = 'COLOR',
		default = (0.8, 0.8, 0.8),
		soft_min = 0,
		soft_max = 1
		)
	mat.pbrt3_matte_kdscale = FloatProperty(
		name = "Kd Scale",
		description = "Scaling factor for Kd",
		default = 1
		)
		
	mat.pbrt3_matte_sigma = FloatProperty(
		name = "Sigma",
		description = "The sigma parameter for the Oren-Nayar model, in degrees. If this is zero, the surface exhibits pure Lambertian reflection.",
		default = 0
		)
		
	# metal
	mat.pbrt3_metal_eta = FloatProperty(
		name = "eta",
		description = "Index of refraction to use in computing the material's reflectance.",
		default = 0.216, # copper at 650nm
		soft_min = 0,
		soft_max = 10
		)
		
	mat.pbrt3_metal_k = FloatProperty(
		name = "k",
		description = "Absorption coefficient to use in computing the material's reflectance.",
		default = 3.64, # copper at 650nm
		soft_min = 0,
		soft_max = 10
		)
		
	mat.pbrt3_metal_roughness = FloatProperty(
		name = "Roughness",
		description = "Roughness of the material's microfacet distribution. Smaller values become increasingly close to perfect specular reflection. This value should be between zero and one.",
		default = 0.01,
		soft_min = 0,
		soft_max = 1
		)
	
	
	# glass
	mat.pbrt3_glass_kr = FloatVectorProperty(
		name = "Kr",
		description = "The reflectivity of the surface.",
		subtype = 'COLOR',
		default = (0.8, 0.8, 0.8),
		soft_min = 0,
		soft_max = 1
		)
	mat.pbrt3_glass_krscale = FloatProperty(
		name = "Kr Scale",
		description = "Scaling factor for Kr",
		default = 1
		)
		
	mat.pbrt3_glass_kt = FloatVectorProperty(
		name = "Kt",
		description = "The transmissivity of the surface.",
		subtype = 'COLOR',
		default = (0.8, 0.8, 0.8),
		soft_min = 0,
		soft_max = 1
		)
	mat.pbrt3_glass_ktscale = FloatProperty(
		name = "Kt Scale",
		description = "Scaling factor for Kt",
		default = 1
		)
		
	mat.pbrt3_glass_index = FloatProperty(
		name = "index",
		description = "The index of refraction of the inside of the object. (pbrt3 implicitly assumes that the exterior of objects is a vacuum, with IOR of 1.)",
		default = 1.5
		)
	
	# mirror
	mat.pbrt3_mirror_kr = FloatVectorProperty(
		name = "Kr",
		description = "The reflectivity of the mirror. This value can be used to make colored or dim reflections.",
		subtype = 'COLOR',
		default = (0.9, 0.9, 0.9),
		soft_min = 0,
		soft_max = 1
		)
	
	
	# area light
	mat.pbrt3_light_islight = BoolProperty(
		name = "Is Light",
		description = "Whether this material emits light or not",
		default = False
		)
	
	mat.pbrt3_light_l = FloatVectorProperty(
		name = "L",
		description = "The amount of emitted radiance at each point and emitted direction.",
		subtype = 'COLOR',
		default = (1, 1, 1),
		soft_min = 0,
		soft_max = 1
		)
	mat.pbrt3_light_lscale = FloatProperty(
		name = "L Scale",
		description = "Scaling factor for L",
		default = 1
		)

		

class ObjectPanel(bpy.types.Panel):
	bl_idname = "PBRT3_OBJECT_PANEL"
	bl_context = 'object'
	bl_label = "pbrt3"
	bl_space_type = 'PROPERTIES'
	bl_region_type = 'WINDOW'
	
	def draw(self, context):
		# some definitions, that will make our lives easier
		m = context.object
		if m is None:
			return
		
		r = self.layout.row()
		r.prop(m, "pbrt3_semantic")
		

def RegisterObjectProperties():
	obj = bpy.types.Object
	from bpy.props import EnumProperty
	
	obj.pbrt3_semantic = EnumProperty(
		name = "Object Semantic",
		description = "for specific rendering obtimizations",
		items = (	('0', 'Default', ''),
					('10', 'NLoS Reflector', ''),
					('11', 'NLoS Object', ''),
				),
		default = '0'
		)



class RenderPanel(bpy.types.Panel):
	bl_idname = "PBRT3_RENDER_PANEL"
	bl_context = 'render'
	bl_label = "pbrt3"
	bl_space_type = 'PROPERTIES'
	bl_region_type = 'WINDOW'
	
	def draw(self, context):
		# some definitions, that will make our lives easier
		l = self.layout
		
		l.prop(context.scene, "pbrt3_integrator")
		l.prop(context.scene, "pbrt3_samples")
		l.prop(context.scene, "pbrt3_max_bounces")
		if context.scene.pbrt3_integrator == "transientpath":
			l.prop(context.scene, "pbrt3_time_bins")
			l.prop(context.scene, "pbrt3_t_min")
			l.prop(context.scene, "pbrt3_t_max")
			l.prop(context.scene, "pbrt3_ignore_distance_to_camera")
		


def register():
	bpy.utils.register_class(ExportPbrt3)
	bpy.utils.register_class(MaterialPanel)
	bpy.utils.register_class(ObjectPanel)
	bpy.utils.register_class(RenderPanel)
	bpy.utils.register_class(Pbrt3Renderer)
	
	RegisterMaterialProperties()
	RegisterObjectProperties()
	
	from bl_ui import properties_render, properties_material, properties_data_lamp, properties_data_camera
	properties_render.RENDER_PT_render.COMPAT_ENGINES.add(Pbrt3Renderer.bl_idname)
	#properties_render.RENDER_PT_output.COMPAT_ENGINES.add(Pbrt3Renderer.bl_idname)
	properties_render.RENDER_PT_dimensions.COMPAT_ENGINES.add(Pbrt3Renderer.bl_idname)
	#properties_material.MATERIAL_PT_preview.COMPAT_ENGINES.add(Pbrt3Renderer.bl_idname)
	#properties_material.MATERIAL_PT_diffuse.COMPAT_ENGINES.add(Pbrt3Renderer.bl_idname)
	properties_material.MATERIAL_PT_context_material.COMPAT_ENGINES.add(Pbrt3Renderer.bl_idname)
	properties_data_lamp.DATA_PT_lamp.COMPAT_ENGINES.add(Pbrt3Renderer.bl_idname)
	properties_data_camera.DATA_PT_lens.COMPAT_ENGINES.add(Pbrt3Renderer.bl_idname)
	# TODO: add Camera Depth-of-Field at some point.
	
	# add to export menu
	bpy.types.INFO_MT_file_export.append(MenuExport)
	
	# Render Properties
	bpy.types.Scene.pbrt3_integrator = bpy.props.EnumProperty(name = "Integrator", items = [("path", "path", ""), ("transientpath", "transientpath", "")], default = "path")
	bpy.types.Scene.pbrt3_samples = bpy.props.IntProperty(name = "Samples", default = 8)
	bpy.types.Scene.pbrt3_max_bounces = bpy.props.IntProperty(name = "Max Bounces", default = 7)
	bpy.types.Scene.pbrt3_time_bins = bpy.props.IntProperty(name = "Time Bins", default = 32)
	bpy.types.Scene.pbrt3_t_min = bpy.props.FloatProperty(name = "t_min", default = 0.0)
	bpy.types.Scene.pbrt3_t_max = bpy.props.FloatProperty(name = "t_max", default = 100.0)
	bpy.types.Scene.pbrt3_ignore_distance_to_camera = bpy.props.BoolProperty(name = "Ignore distance to camera", default = True)	
	
	
	
def unregister():
	# remove from export menu
	bpy.types.INFO_MT_file_export.remove(MenuExport)
	
	bpy.utils.unregister_class(Pbrt3Renderer)
	bpy.utils.unregister_class(MaterialPanel)
	bpy.utils.unregister_class(ExportPbrt3)
   

if __name__ == "__main__":

	register()
	
	invoke = 0 # 0: nothing, 1: start exporter, 2: direct export
	
	if 1 == invoke:
		# normal invoke
		bpy.ops.export_scene.pbrt3('INVOKE_DEFAULT')
	elif 2 == invoke:
		# just do a default export
		import subprocess
		f = open("ExporterDebug.pbrt", 'w', encoding='utf-8')
		f.write(GenerateSceneString(bpy.context, "Scene.exr"))
		f.close()
		#subprocess.call([pbrt3Path, "Exporter.pbrt"], shell=True)
