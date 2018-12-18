#!/usr/bin/env python

#PC2_Data_Extract.py

#Subscribe to velodyne_points topic in CloudNode in velodyne_pointcloud package.
#Extract xyz and intensity data from PointCloud2 type message
#Save data for later use

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String, Header

global all_points, time_stamps

all_points = [np.zeros(4)]
time_stamps = [np.zeros(2)]
#Prefix to names of dummy fields added to get correct byte alignment
DummyFieldPrefix = '__'

#Mappings between PointField data types and numpy data types
type_mapping = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')), (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')), (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype2nptype = dict(type_mapping)
#Size in bytes of PointField types
pftype_size = {PointField.INT8:1, PointField.UINT8:1, PointField.INT16:2, PointField.UINT16:2, PointField.INT32:4, PointField.UINT32:4, PointField.FLOAT32:4, PointField.FLOAT64:8}

def field2dtype(fields, point_step):
	'''Convert list of PointFields into numpy record data type
	fields describes what one point entry looks like'''
	fields:
		string name (=name of field)
		uint32 offset (=offset in bytes from start of point struct)
		unint8 datatype (=datatype of field)
		unint32 count (=number of elements in field in each point)
	point_step:
		uint32 that describes length of a point in bytes

	
	offset = 0
	np_dtype_list = []

	for f in fields:
		#increment and add offset to field if necessary
		while offset < f.offset:
			np_dtype_list.append(('%s%d' % (DummyFieldPrefix, offset), np.uint8))
			offset+=1
		
		dtype = pftype2nptype[f.datatype]	#Use dictionary to determine corresponding numpy data type
		#Shouldn't have to worry about this
		if f.count != 1:
			dtype = np.dtype((dtype,f.count))

		np_dtype_list.append((f.name, dtype))
		offset += pftype_size[f.datatype]*f.count	#increment offset to offset of next field
	
	#increment offset and add offset between points if necessary
	while offset<point_step:
		np_dtype_list.append(('%s%d' % (DummyFieldPrefix, offset), np.uint8))
		offset += 1

	return np_dtype_list

def pointcloud2_to_array(pc2_msg):
	'''Converts rospy PointCloud2 message to a numpy rec-array. 
	'''

	#Create numpy rec-array type equivalent to type of point cloud
	dtype_list = field2dtype(pc2_msg.fields, pc2_msg.point_step)
	
	#Parse point cloud into numpy array of proper data type
	pc_arr = np.fromstring(pc2_msg.data, dtype_list)

	#Remove dummy fields if added
	pc_arr = pc_arr[[fname for fname, _type in dtype_list if not (fname[:len(DummyFieldPrefix)]==DummyFieldPrefix)]]

	return np.reshape(pc_arr, (pc2_msg.height, pc2_msg.width))

def get_xyzi(pc_array, remove_nans=True, dtype=np.float):
	'''Extracts x,y,z, and intensity folumns from point cloud rec array and returns 4xN matrix
	'''
	
	#Remove shitty points
#	if remove_nans:
#		mask = np.isfinite(pc_array['x']) & np.isfinite(pc_array['y']) & np.isfinite(pc_array['z']) & np.isfinite(pc_array['i'])
#		pc_array = pc_array[mask]

	#Pull out x,y,z, and intensity values
#	print(pc_array.shape[1])
	points = np.zeros((pc_array.shape[1],4), dtype=dtype)
	points[...,0] = pc_array['x']
	points[...,1] = pc_array['y']
	points[...,2] = pc_array['z']
	points[...,3] = pc_array['intensity']
	#Row of zeros between each scan
#	print(points.shape)
	nu = np.zeros(4)
#	print(nu.shape)
	points = np.concatenate((points, [nu]), axis=0)

	return points

def pointcloud2_to_xyzi_array(pc_msg, remove_nans=True):
	return get_xyzi(pointcloud2_to_array(pc_msg), remove_nans=remove_nans)

def save_xyzi(xyzi_array):
	f=open('xyzi_data.csv', 'w')
	np.savetxt(f,xyzi_array,fmt='%1.10f',delimiter=',')
	print('XYZI data has been save to xyzi_data.csv')

def save_time_stamps(stamps):
	f=open('time_stamps.csv', 'w')
	np.savetxt(f,stamps,fmt='%1.10d', delimiter=',')
	print('Time stamps have been saved to time_stamps.csv')

def Callback(data):
	global all_points, time_stamps
	print('Test')
	all_points = np.concatenate((all_points, pointcloud2_to_xyzi_array(data)), axis=0)
#	print(all_points)
	time_stamps = np.append(time_stamps, [[data.header.stamp.secs, data.header.stamp.nsecs]], axis=0)
#	print(time_stamps)

def data_dump_node():
	'''Initialize the ros node and subscribe to the velodyne_points topic
	'''

	rospy.init_node('data_dump', anonymous=True)
	rospy.Subscriber("velodyne_points", PointCloud2, Callback)
	rospy.spin()

if __name__=='__main__':
	data_dump_node()
	while not rospy.is_shutdown():
		pass

	save_xyzi(all_points)
	save_time_stamps(time_stamps)


