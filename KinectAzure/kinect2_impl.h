#ifdef SendMessage
#undef SendMessage
#endif


#include <RobotRaconteur.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include "RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h"
#include "RobotRaconteurCompanion/YamlConverters/YAMLconverter__importer.h"
//#include "RobotRaconteurCompanion/YamlConverters/YAMLconverter__com.robotraconteur.imaging.camerainfo.h"
//#include "RobotRaconteurCompanion/YamlConverters/YAMLconverter__com.robotraconteur.pointcloud.sensor.h"
#include <k4a/k4a.h>
//#include "k4amicrophone.h"
#include <k4abt.h>
#include <k4arecord/playback.h>
#include "robotraconteur_generated.h"

#include <Windows.h>
//#include <Kinect.h>
#include <iostream>
#include <boost/enable_shared_from_this.hpp>
#include <vector>
#include <string>
#include <cmath>
#include "yaml-cpp/yaml.h"
#include <boost/uuid/uuid_io.hpp>
//#include <pcl/io/boost.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <opencv2/opencv.hpp>

#pragma once
using namespace RobotRaconteur;
using namespace boost;
namespace geometry = com::robotraconteur::geometry;
namespace pointcloud = com::robotraconteur::pointcloud;
namespace imaging = com::robotraconteur::imaging;
namespace image = com::robotraconteur::image;
namespace sensor = com::robotraconteur::sensor;
namespace identifier = com::robotraconteur::identifier;
namespace device = com::robotraconteur::device;
namespace imu = com::robotraconteur::imu;

class Kinect2_impl;

typedef union
{
	/** XYZ or array representation of vector. */
	struct _xyz
	{
		int16_t x; /**< X component of a vector. */
		int16_t y; /**< Y component of a vector. */
		int16_t z; /**< Z component of a vector. */
	} xyz;         /**< X, Y, Z representation of a vector. */
	int16_t v[3];    /**< Array representation of a vector. */
} PointCloudPixel_int16x3_t;

class KinectPointCloud : virtual public pointcloud::sensor::PointCloudSensor_default_impl 
{
	
public:
	KinectPointCloud(Kinect2_impl* reference);
	~KinectPointCloud();
	
	//pointcloud::sensor::PointCloudSensorDataPtr getPointCloud();
	pointcloud::sensor::PointCloudSensorDataPtr getPointCloudThreaded(int height,int width, PointCloudPixel_int16x3_t *pointcloudtemp);
	virtual void set_active(RobotRaconteur::rr_bool active) override;
	virtual RobotRaconteur::rr_bool get_active() override;
	virtual void set_point_cloud_sensor_data(PipePtr< pointcloud::sensor::PointCloudSensorDataPtr> value);
	virtual pointcloud::sensor::PointCloudSensorInfoPtr get_point_sensor_info();
	virtual device::DeviceInfoPtr get_device_info();
	void send_data(pointcloud::sensor::PointCloudSensorDataPtr pointcloud);
	Kinect2_impl* reference;
	std::vector<k4a_float3_t> m_cloudPoints;
	device::DeviceInfoPtr device_info;
	pointcloud::sensor::PointCloudSensorInfoPtr sensor_infos;

private:
	bool point_cloud_active;
	
	//pointcloud::sensor::PointCloudSensorInfo point_cloud_info;

};

class KinectIMU : virtual public imu::ImuSensor_default_impl {

public:
	KinectIMU(Kinect2_impl* reference);
	//~KinectIMU();
	//imu::ImuStatePtr getIMUThreaded();
	virtual void set_active(RobotRaconteur::rr_bool active) override;
	virtual RobotRaconteur::rr_bool get_active() override;
	virtual sensor::SensorInfoPtr get_sensor_info() override;
	virtual void set_sensor_data(PipePtr<sensor::SensorDataPtr> value);
	void send_data(k4a_imu_sample_t sample);

	Kinect2_impl* reference;
	sensor::SensorInfoPtr sensorinfo;

private:
	bool imu_active;
	int seqno;
	
};


class KinectMultiCamera : virtual public imaging::MultiCamera_default_impl
{
	class Kinect_Camera;
	
public:
	KinectMultiCamera(Kinect2_impl* reference);
	~KinectMultiCamera();
	//Kinect_Camera* cameras[4];
	Kinect2_impl* reference;
	virtual imaging::CameraPtr get_cameras(int32_t ind) override;
	std::vector<boost::shared_ptr<KinectMultiCamera::Kinect_Camera> > cameras;
	virtual imaging::camerainfo::MultiCameraInfoPtr get_multicamera_info() override;
	imaging::camerainfo::MultiCameraInfoPtr multicamera_info;
	
	
	
	class Kinect_Camera : public imaging::Camera_default_impl, public boost::enable_shared_from_this<Kinect_Camera>
	{

		int camera_num;
		
		
		image::ImagePtr imagedata;
	public:
		Kinect_Camera(int camera_num, KinectMultiCamera* reference);
		~Kinect_Camera();
		int image_width, image_height, image_step;
		int image_encoding;
		void send_data(image::ImagePtr image_data);
		virtual  image::ImagePtr capture_frame();
		virtual void start_streaming();
		virtual void stop_streaming();
		virtual void set_frame_stream(PipePtr< image::ImagePtr> value);
		KinectMultiCamera* multicam_reference;
		
		imaging::camerainfo::CameraInfoPtr camera_info;
		virtual imaging::camerainfo::CameraInfoPtr get_camera_info() override;
		
		
	private:
		bool streaming;
		//imaging::camerainfo::CameraInfo camera_info;
	};
	



};

class KinectBodyTracker : virtual public edu::rpi::robotics::kinect::Kinect_default_impl
{

	
	
public:
	KinectBodyTracker(Kinect2_impl* reference);
	~KinectBodyTracker();
	Kinect2_impl* reference;
	virtual RR_INTRUSIVE_PTR<RobotRaconteur::RRArray<uint64_t > > getTrackedBodyIDs();
	//virtual RR_INTRUSIVE_PTR<sensors::kinect2::KinectBody > getDetectedBody(int32_t index);
	virtual void getDetectedBodyThreaded(k4abt_frame_t body_frame, int body_num);
	virtual RR_INTRUSIVE_PTR <sensor::SensorInfo> get_sensor_info() override;
	virtual void set_active(RobotRaconteur::rr_bool active) override;
	virtual RobotRaconteur::rr_bool get_active() override;
	virtual device::DeviceInfoPtr get_device_info();
	virtual void set_kinect_body_sensor_data(PipePtr<edu::rpi::robotics::kinect::KinectBodiesSensorDataPtr >  value);
	sensor::SensorInfoPtr sensor_info_stored;
	device::DeviceInfoPtr device_info;

private:
	bool tracking_active;
	//sensor::SensorInfo sensor_info_stored;
};




class Kinect2_impl 
{
	
	
public:
	RR_SHARED_PTR < KinectPointCloud> pointcloud;
	RR_SHARED_PTR < KinectMultiCamera> multicamera;
	//multicamera would be parent device and tracker and pointcloud would be dependent on that
	RR_SHARED_PTR < KinectBodyTracker> tracker;
	RR_SHARED_PTR <KinectIMU> imu;
	int color_image_size, body_index_size, depth_image_size, infrared_image_size;
	int color_image_width, color_image_height;
	int depth_image_width, depth_image_height;
	uint8_t *color_image_data;
	uint8_t *bodyindex_image_data;
	uint16_t *depth_image_data;
	uint16_t *infrared_image_data;
	//uint16_t *longexposure_infrared_image_data;
	//uint16_t *pointcloud_temp;
	boost::mutex mtx_;
	uint64_t tracked_body_ids[6];
	device::DeviceInfo device_info;

	int enabledSources;
	Kinect2_impl();
	~Kinect2_impl();
	k4a_device_t kinect;
	HRESULT StartupKinect();
	HRESULT ShutdownKinect();
	//virtual sensors::kinect2::ImagePtr getCurrentColorImage();
	//virtual sensors::kinect2::DepthImagePtr getCurrentDepthImage();
	//virtual sensors::kinect2::DepthImagePtr getCurrentInfraredImage();
	//virtual sensors::kinect2::ImagePtr getCurrentBodyIndexImage();
	//virtual sensors::kinect2::DepthImagePtr getCurrentLongExposureInfraredImage();
	

private:
	k4abt_tracker_t kinect_body_tracker;
	//RGBQUAD *color_image_data;
	k4a_imu_sample_t imu_sample;
	k4a_image_t pointcloudimagetemp;
	k4a_calibration_t sensor_calibration;
	k4a_device_configuration_t config;
	
	k4a_transformation_t transform;
	
	
	bool thread_exit;
	int TIMEOUT_IN_MS;
	
	
	//ICoordinateMapper *coordinate_mapper;

	//WAITABLE_HANDLE h_event;
	
	boost::thread t1;

	void CaptureArrived(k4a_capture_t* capture);

	void backgroundPollingThread();

	template<class Interface> inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
};

class Kinect_impl_playback
{


public:
	RR_SHARED_PTR < KinectPointCloud> pointcloud;
	RR_SHARED_PTR < KinectMultiCamera> multicamera;
	//multicamera would be parent device and tracker and pointcloud would be dependent on that
	//RR_SHARED_PTR < KinectBodyTracker> tracker;
	RR_SHARED_PTR <KinectIMU> imu;
	int color_image_size, body_index_size, depth_image_size, infrared_image_size;
	int color_image_width, color_image_height;
	int depth_image_width, depth_image_height;
	uint8_t* color_image_data;
	uint8_t* bodyindex_image_data;
	uint16_t* depth_image_data;
	uint16_t* infrared_image_data;
	//uint16_t *longexposure_infrared_image_data;
	//uint16_t *pointcloud_temp;
	boost::mutex mtx_;
	//uint64_t tracked_body_ids[6];
	device::DeviceInfo device_info;

	//int enabledSources;
	Kinect_impl_playback(char* filename);
	~Kinect_impl_playback();
	k4a_device_t kinect;
	HRESULT StartupKinect();
	



private:
	//k4abt_tracker_t kinect_body_tracker;
	//RGBQUAD *color_image_data;
	k4a_imu_sample_t imu_sample;
	k4a_image_t pointcloudimagetemp;
	//k4a_calibration_t sensor_calibration;
	//k4a_device_configuration_t config;

	k4a_transformation_t transform;
	k4a_playback_t playback;
	

	bool thread_exit;
	int TIMEOUT_IN_MS;


	//ICoordinateMapper *coordinate_mapper;

	//WAITABLE_HANDLE h_event;

	boost::thread t1;

	void CaptureArrived(k4a_capture_t* capture);

	void backgroundPollingThread();

	template<class Interface> inline void SafeRelease(Interface*& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
};