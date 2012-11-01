#!/usr/bin/env python

import sys
import os
import fnmatch
from tvtk.api import tvtk
from numpy.random import randint, rand, permutation
import numpy as np
import time

from optparse import OptionParser

num_objects_in_scene = 8
model_center_square = 1.5

camera_height = 1.5
camera_height_delta = 0.5

camera_rotation_delta = 5

baseline = 0.075
focal_length = 585.0

width = 640
height = 480


pcd_header = """
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH %d
HEIGHT 1
VIEWPOINT %f %f %f 0 0 0 1
POINTS %d
DATA ascii
"""

y_grid, x_grid = np.mgrid[-240:240,-320:320].astype(np.float32)

def compute_points(depth_image):
	points = np.empty((depth_image.shape[0], depth_image.shape[1], 4), dtype=np.float32)
	points[:,:,2] = depth_image
	points[:,:,1]= y_grid * points[:,:,2] / focal_length
	points[:,:,0]= x_grid * points[:,:,2] / focal_length
	points[:,:,3]=1
	return points

def get_models_from_dir(models_dir):
	classes = os.listdir(models_dir)
	models = {}
	for c in classes:
		model_files = [ f for f in os.listdir(models_dir + '/' + c) if fnmatch.fnmatch(f,'*.vtk')]
		models[c] = {}
		for m in model_files:
			r = tvtk.PolyDataReader()
			r.file_name = models_dir + '/' + c + '/' + m
			r.update()
			models[c][m] = tvtk.PolyDataMapper(input=r.output)
	return models


def get_render_window(options):
	ren = tvtk.Renderer(background=(0.0, 0.0, 0.0))
	rw = tvtk.RenderWindow(size=(width,height))
	rw.off_screen_rendering = 0
	rw.add_renderer(ren)
	rw.multi_samples = 0
	ac = ren.active_camera
	ac.view_angle = 44.61
	
	ac.position = [ options.shift, -options.distance , options.height]
	ac.view_up = [ 0, 0 , 1]
	ac.focal_point = [ options.shift, 0, options.height]
	ac.pitch(options.tilt)
	
	return rw

	

def save_pointcloud(rw, options, classname, modelname, i, actor):
	narr = np.empty((height,width), dtype=np.float32)
	rw.get_zbuffer_data(0,0,width-1,height-1,narr)

	narr = np.flipud(narr)

	cr = rw.renderers[0].active_camera.clipping_range
	mask = (narr == 1.0)

	# Z buffer stores z values in log scale. Getting z coordinate in world coordinates.
	narr = (narr - 0.5) * 2.0
	narr = 2*cr[1]*cr[0]/(narr*(cr[1]-cr[0])-(cr[1]+cr[0]))
	narr = -narr

	narr += np.random.normal(0, options.noise_std, (480,640))
	narr[mask] = 0
	
	points = compute_points(narr)
	points = points[np.nonzero(narr > 0)]

	ac = rw.renderers[0].active_camera
	m = ac.view_transform_matrix.to_array()
	transform_matrix = np.linalg.inv(np.dot(np.array([[1,0,0,0],[0,-1, 0,0],[0,0,-1,0],[0,0,0,1]]), m))
	transform_matrix = np.dot(np.array([[0,1,0,0],[-1, 0, 0,0],[0,0,1,0],[0,0,0,1]]), transform_matrix)
	points = np.dot(transform_matrix, points.T)
	points = points.T
	points[:, 2] -= actor.center[2]
	

	filename = options.output_dir + classname + '/' +  modelname[0:-4] + '/'
	filename += 'rotation' + str(i*360/options.num_views) + '_distance' + str(options.distance) 
	filename += '_tilt' + str(options.tilt) + '_shift' + str(options.shift) + '.pcd'
	
	f = open(filename,'w')
	f.write(pcd_header % (points.shape[0], transform_matrix[0,3], transform_matrix[1,3], transform_matrix[2,3], points.shape[0]))
	for i in range(points.shape[0]):
		f.write("%f %f %f\n" % (points[i,0], points[i,1], points[i,2]))
	f.close()
	


if __name__ == '__main__':
	parser = OptionParser()
	parser.add_option("--input_dir", type="string", dest="input_dir")
	parser.add_option("--output_dir", type="string", dest="output_dir")
	parser.add_option("--height", type="float", dest="height", default=1.5)
	parser.add_option("--noise_std", type="float", dest="noise_std", default=0.0001)
	parser.add_option("--distance", type="float", dest="distance", default=4.0)
	parser.add_option("--tilt", type="float", dest="tilt", default=-30.0)
	parser.add_option("--shift", type="float", dest="shift", default=0)
	parser.add_option("--num_views", type="int", dest="num_views", default=6)

	options, args = parser.parse_args()

	
	if options.input_dir == None or options.output_dir == None:
		print 'Please specify input and output directories'
		sys.exit(-1)

	models = get_models_from_dir(options.input_dir)
	rw = get_render_window(options)

	if not os.path.exists(options.output_dir):
    		os.makedirs(options.output_dir)
	
	for classname, models_map in models.iteritems():
		class_dir = options.output_dir + classname
		if not os.path.exists(class_dir):
    			os.makedirs(class_dir)
		for modelname, mapper in models_map.iteritems():
			model_dir = options.output_dir + classname + '/' + modelname[0:-4]
			if not os.path.exists(model_dir):
    				os.makedirs(model_dir)
			actor = tvtk.Actor(mapper=mapper)
			angle = 360.0/options.num_views
			rw.renderers[0].add_actor(actor)
			for i in range(options.num_views):
				actor.rotate_z(angle)
				rw.render()
				save_pointcloud(rw, options, classname, modelname, i, actor)
			rw.renderers[0].remove_actor(actor)
				
	
	

 
