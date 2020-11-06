
from RobotRaconteur.Client import *
import numpy as np
import cv2
import time
import cv2.aruco as aruco
#import eigen as eigen3


def DepthImageToMat(image):
    frame2=image.data.reshape([image.height, image.width])#, order='C')
    #frame2=np.empty([image.width,image.height])
    #pclimage=np.zeros((image.width,image.height), dtype=np.float32)
    #point=np.zeros(3,dtype=np.float32)
    cx=image.width/2
    cy=image.height/2
    for i in range(image.height):
        for j in range(image.width):
	        frame2[i][j]=frame2[i][j]/4500*65535
            
            
            
    return frame2

def rawDepthToMeters(value):
    if(value < 2047):
        return 1.0/(i*-0.0030711016+3.3309495161)
    else:
        return 0.0
        
        
def new_frame(pipe_ep):
    #print("new frame arrived")

    image=pipe_ep.ReceivePacket()
    #frame2=image.data.reshape([image.image_info.height, image.image_info.width, 3], order='C')
    #frame2=cv2.imdecode(np.fromstring(image,dtype=np.uint8),cv2.IMREAD_COLOR)
    #cv2.imshow('frame',frame2)
    pointcloud=image.point_cloud.points
    for i in pointcloud:
        print(i.x)
        
        
    #cv2.waitKey()
    #print(image.image_info.height)
    #print(image.data[0])
    
    
def framerestyle(pipe_ep):
    image=pipe_ep.ReceivePacket()
    image2=cv2.imdecode(image.data,cv2.IMREAD_COLOR)
    scale_percent = 60 # percent of original size
    width = int(image2.shape[1] * scale_percent / 100)
    height = int(image2.shape[0] * scale_percent / 100)
    dim = (width, height)
    # resize image
    print("bye")
    resized = cv2.resize(image2, dim, interpolation = cv2.INTER_AREA)
    cv2.imshow('frame', resized)
    cv2.waitKey(10)
    
    
    
    

def bodytracker(pipe_ep):
    data=pipe_ep.ReceivePacket()
    print(data.tracking_id)
    print(data.joint_tracking_state[1])
    
def pointcloudreceive(pipe_ep):
    data=pipe_ep.ReceivePacket()
    print("got new data")
    print(data.point_cloud.points[200])
   
def imu_data_send(pipe_ep):
    data=pipe_ep.ReceivePacket()
    print(data.data)

def main():
    
    #url='tcp://localhost:8888/sensors.kinect2/KinectTracker'
    url='rr+local:///?nodeid=8186d173-72de-414c-b408-948b68eaaee1&service=KinectMultiCamera'
    #url='rr+tcp://[fe80::416f:af5d:608f:269d]:8888/?nodeid=8186d173-72de-414c-b408-948b68eaaee1&service=KinectPointCloud'
    #url='rr+tcp://[fe80::416f:af5d:608f:269d]:8888/?nodeid=8186d173-72de-414c-b408-948b68eaaee1&service=KinectIMU'
    kinect=RRN.ConnectService(url)
    #kinect.active=True;
    #p=kinect.point_cloud_sensor_data.Connect(-1)
    #p.PacketReceivedEvent+=pointcloudreceive
    camera=kinect.get_cameras(0)
    camera.start_streaming()
    #kinect.active_body_tracking=True
    #p=kinect.sensor_data.Connect(-1)
    #p.PacketReceivedEvent+=imu_data_send
    #kinect_info=kinect.point_sensor_info
    #print(kinect_info)
    #print(kinect_info.device_info.manufacturer.uuid)
    #print("hello")
    
    #p=kinect.kinect_body_sensor_data.Connect(-1)
    #p.PacketReceivedEvent+=bodytracker
    p=camera.frame_stream.Connect(-1)
    p.PacketReceivedEvent+=framerestyle
    #color_cam=kinect.get_cameras(0)
    #color_cam.start_streaming()
    #p=color_cam.frame_stream.Connect(-1)
    #p=kinect.point_cloud_sensor_data.connect(-1)
    #p.PacketReceivedEvent+=new_frame
    #sensors_enabled=kinect.SensorsEnabled()
    #sensors_enabled.Depth=1
    #sensors_enabled.Color=1
    
    #sensors_enabled.Body=1
    #sensors_enabled.BodyIndex=1
    
    #sensors_enabled.Infrared=1
    #sensors_enabled.LongExposureInfrared=1
    
    #result=kinect.EnableSensors(sensors_enabled)
    #depthlookup=np.zeros(2048,dtype=np.float32)
    #for i in range(depthlookup.shape(0)):
    #    depthlookup[i]=rawDepthToMeters(i)
    #if(not result):
    #    kinect.DisableSensors()
    #    kinect.EnableSensors(sensors_enabled)
    
    #cv2.waitKey()
    input("Press enter to exit")
    #depth=kinect.getCurrentDepthImage()
    #color=kinect.getCurrentColorImage()
    #cloud=kinect.getPointCloud()
    #print(depth.data)
    #print(color.data)
    #cv2.imshow('image',depth)
    
    """
    cloud2=np.empty([len(cloud.points),3])
    
    #print(pclimage.shape)
    for i in range(len(cloud.points)-1):
        cloud2[i]=[cloud.points[i][0],cloud.points[i][1],cloud.points[i][2]]
    
    
    p=pcl.PointCloud()
    #p.width=512
    #p.height=424
    
    p.from_array(cloud2.astype(np.float32))
    
    visual = pcl.pcl_visualization.CloudViewing(b'PCL OpenNI Viewer')


    visual.ShowMonochromeCloud(p, b'cloud')
    fil = p.make_passthrough_filter()
    fil.set_filter_field_name("z")
    #sets detection distance forward of the kinect
    fil.set_filter_limits(0, 1.5)
    cloud_filtered = fil.filter()
    print(cloud_filtered.size)
    seg=cloud_filtered.make_segmenter_normals(ksearch=50)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(100)
    #sets tightness of segmenter for detecting planar object
    seg.set_distance_threshold(0.03)
    #creates list of indexes for points that meet model and gives coefficients for the plane defined by ax+by+cz+d=0
    indices, model=seg.segment()
    print(model)
    cloud_plane=cloud_filtered.extract(indices, negative=False)
    noplane_cloud=cloud_filtered.extract(indices, negative=True)
    #print(indices)
    segmenter=noplane_cloud.make_segmenter_normals(searchRadius=0.03)
    blank_image = np.zeros((depth.height,depth.width,1), np.uint8)
    #for i in range(len(indices)):
    #    point=cloud_filtered.__getitem__(indices[i])
    #    w=point[0]
    #    h=point[1]
        
    #    blank_image[h][w]=255
    
    
    segmenter.set_model_type(pcl.SACMODEL_CYLINDER)
    segmenter.set_normal_distance_weight(0.1)
    segmenter.set_method_type(pcl.SAC_RANSAC)
    segmenter.set_max_iterations(10000)
    segmenter.set_distance_threshold(0.05)
    segmenter.set_radius_limits(0, 0.1)
    #axis=eigen3.Vector3f(model[0],model[1],model[2])
    #segmenter.set_axis(model[0],model[1],model[2])
    #segmenter.set_distance_threshold(0.1)
    #indices2, model2=segmenter.segment()
    #cloud_plane2=cloud_filtered.extract(indices2, negative=False)
    #[inliers_cylinder, coefficients_cylinder] = segmenter.segment()
    #cloud_cylinder = noplane_cloud.extract(inliers_cylinder, negative=False)
    #print(inliers_cylinder)
    #outputCloud = pcl.PointCloud()
    #crophull=cloud_filtered.make_crophull()
    #crophull.set_InputCloud(cloud_plane)
    #ability to capture waypoints and send them back in python
    #crophull.Filtering(outputCloud)
    #viewer = pcl.pcl_visualization.PCLVisualizering()
    #viewer.AddPointCloud(p, b'scene_cloud', 0)
    #viewer.SpinOnce()
    #viewer.RemovePointCloud(b'scene_cloud', 0)
    
    
    
    #pic=np.array(picture)
    #print(picture.shape)
    
    #print(picture)
    cv2.imshow('image',blank_image)
    cv2.waitKey()
	"""

if __name__ == "__main__":
	main()
