
#include "kinect2_impl.h"
#include <chrono>
//#include "yaml_loader.h"
//#include <Yaml_loader_enums.h>

/*
Camera numbers:
0=Color
1=Infrared
2=Depth
3=Body Index
4=Point Cloud
5=Body Joint
6=Preview Stream
*/

namespace RR = RobotRaconteur;
namespace pointcloud = com::robotraconteur::pointcloud;
namespace imaging = com::robotraconteur::imaging;
namespace image = com::robotraconteur::image;
namespace sensor = com::robotraconteur::sensor;
namespace identifier = com::robotraconteur::identifier;
namespace uuid = com::robotraconteur::uuid;
namespace resource = com::robotraconteur::resource;

KinectPointCloud::KinectPointCloud(Kinect2_impl* reference) {
	this->reference = reference;
	this->point_cloud_active = false;
	
}

KinectPointCloud::~KinectPointCloud() {
	this->point_cloud_active = false;
	
}

device::DeviceInfoPtr KinectPointCloud::get_device_info() {
	return this->device_info;
}


void KinectPointCloud::set_active(RobotRaconteur::rr_bool active) {
	//std::cout << "setting point cloud active" << std::endl;
	if (this->point_cloud_active != active.value) {
		this->point_cloud_active = active.value;
		if (active.value) {
			this->reference->enabledSources += 16;
		}
		if (!active.value) {
			this->reference->enabledSources -= 16;
		}
	}
}

RobotRaconteur::rr_bool KinectPointCloud::get_active() {
	return this->point_cloud_active;
}



void KinectPointCloud::set_point_cloud_sensor_data(PipePtr<pointcloud::sensor::PointCloudSensorDataPtr> value) {
	this->rrvar_point_cloud_sensor_data = RR_MAKE_SHARED<RobotRaconteur::PipeBroadcaster<pointcloud::sensor::PointCloudSensorDataPtr> >();
	this->rrvar_point_cloud_sensor_data->Init(value);
	this->rrvar_point_cloud_sensor_data->SetMaxBacklog(3);
}

pointcloud::sensor::PointCloudSensorInfoPtr KinectPointCloud::get_point_sensor_info()
{
	return this->sensor_infos;
}

pointcloud::sensor::PointCloudSensorDataPtr KinectPointCloud::getPointCloudThreaded(int height,int width, PointCloudPixel_int16x3_t *pointcloudtemp)
{

	//std::cout << "entered point cloud" << std::endl;
	pointcloud::sensor::PointCloudSensorDataPtr pointclouddata(new pointcloud::sensor::PointCloudSensorData());
	pointcloud::PointCloudfPtr pointcloud(new pointcloud::PointCloudf());
	int depth_image_size = height * width;
	RR::RRNamedArrayPtr<com::robotraconteur::geometryf::Point> cloud = RR::AllocateEmptyRRNamedArray<com::robotraconteur::geometryf::Point>(depth_image_size);
	com::robotraconteur::geometryf::Point* cloud_ptr = &cloud->at(0);
	//std::cout << pointcloudtemp[100].v[0]<< std::endl;
	//std::cout << pointcloudtemp[100].v[1] << std::endl;
	//std::cout << pointcloudtemp[100].v[2] << std::endl;
	//uint16_t* pointcloud_temp = this->reference->pointcloud_temp;
	
	for (int y = 0; y < height; y += 1) {
		for (int x = 0; x < width; x += 1, cloud_ptr++) {
			int pixelindex = y * width + x;
			//std::cout << pixelindex << std::endl;
			cloud_ptr->s.x = pointcloudtemp[pixelindex].v[0]*0.001f;
			cloud_ptr->s.y = pointcloudtemp[pixelindex].v[1]*0.001f;
			cloud_ptr->s.z = pointcloudtemp[pixelindex].v[2]*0.001f;
			//cloud_ptr++;
			//std::cout << this->reference->pointcloud_temp[y] << std::endl;
		}
	}
	//std::cout << "finished looping" << std::endl;
	pointcloud->width = this->reference->depth_image_width;
	pointcloud->height = this->reference->depth_image_height;
	pointcloud->is_dense = true;
	pointcloud->points = cloud;
	pointclouddata->point_cloud = pointcloud;
	
	
	this->rrvar_point_cloud_sensor_data->SendPacket(pointclouddata);
	//this->rrvar_point_cloud_sensor_data->SendPacket(pointclouddata);
	return pointclouddata;

}

void KinectPointCloud::send_data(pointcloud::sensor::PointCloudSensorDataPtr pointcloud) {
	//this->rrvar->SendPacket(image_data);
}

KinectMultiCamera::KinectMultiCamera(Kinect2_impl* reference) {

	this->reference = reference;
	std::vector<boost::shared_ptr<KinectMultiCamera::Kinect_Camera> > cameras;


	cameras.push_back(boost::make_shared<KinectMultiCamera::Kinect_Camera>(0, this)); //color
	cameras.push_back(boost::make_shared<KinectMultiCamera::Kinect_Camera>(1, this));
	cameras.push_back(boost::make_shared<KinectMultiCamera::Kinect_Camera>(2, this));
	cameras.push_back(boost::make_shared<KinectMultiCamera::Kinect_Camera>(3, this));
	
	

	cameras[0]->image_height = reference->color_image_height;
	cameras[0]->image_width = reference->color_image_width;
	cameras[0]->image_step = 4;
	cameras[0]->image_encoding = K4A_IMAGE_FORMAT_COLOR_MJPG;
	//TODO: Make sure that Body index map size matches depth image size
	cameras[1]->image_height = reference->depth_image_height;
	cameras[1]->image_width = reference->depth_image_width;
	cameras[1]->image_step = 2;
	cameras[1]->image_encoding = 0x2001;

	cameras[2]->image_height = reference->depth_image_height;
	cameras[2]->image_width = reference->depth_image_width;
	cameras[2]->image_step = 2;
	cameras[2]->image_encoding = 0x4000;


	cameras[3]->image_height = reference->depth_image_height;
	cameras[3]->image_width = reference->depth_image_width;
	cameras[3]->image_step = 1;
	cameras[3]->image_encoding = 0x2000;


	this->cameras = cameras;
	
}

KinectMultiCamera::~KinectMultiCamera() {


}

KinectMultiCamera::Kinect_Camera::Kinect_Camera(int cam_num, KinectMultiCamera* reference) : imaging::Camera_default_impl()
{
	this->camera_num = cam_num;
	
	//this->imagedata = imagedata;
	this->multicam_reference = reference;
	image_width, image_height, image_step,image_encoding=0;
	this->streaming = false;
	//reference->cameras[cam_num] = this;
	
}

KinectMultiCamera::Kinect_Camera::~Kinect_Camera()
{
	this->stop_streaming();
	

}

image::ImagePtr KinectMultiCamera::Kinect_Camera::capture_frame() {
	boost::mutex::scoped_lock lock(this->multicam_reference->reference->mtx_);
	//boost::lock_guard<boost::mutex> guard(this->multicam_reference->reference->mtx_);
	if (!this->streaming) {
		printf("Error camera must be streaming to capture frame");
		return 0;
	}
	image::ImagePtr imagedata(new image::Image());
	image::ImageInfoPtr image_info(new image::ImageInfo());
	
	image_info->height = this->image_height;
	image_info->width = this->image_width;
	image_info->step = this->image_step;
	imagedata->image_info = image_info;
	switch (camera_num) {
	case 0:
		imagedata->data = AttachRRArrayCopy(this->multicam_reference->reference->color_image_data, this->multicam_reference->reference->color_image_size);
		//std::cout << "sending image" << std::endl;
		break;
	case 1:
		imagedata->data = AttachRRArrayCopy((uint8_t *)this->multicam_reference->reference->infrared_image_data, this->multicam_reference->reference->infrared_image_size);
		break;

	case 2:
		imagedata->data = AttachRRArrayCopy((uint8_t *)this->multicam_reference->reference->depth_image_data, this->multicam_reference->reference->depth_image_size);
		break;
	case 3:
		imagedata->data = AttachRRArrayCopy((uint8_t *)this->multicam_reference->reference->bodyindex_image_data, this->multicam_reference->reference->body_index_size);
		break;

	}
	return imagedata;
}

void KinectMultiCamera::Kinect_Camera::start_streaming() {
	//initialize image data here so that it can be reused
	boost::lock_guard<boost::mutex> guard(this->multicam_reference->reference->mtx_);
	//std::cout << (2^(this->camera_num)) << std::endl;
	if (!this->streaming) {
		this->multicam_reference->reference->enabledSources += pow(2, (this->camera_num));
		this->streaming = true;
	}
}

void KinectMultiCamera::Kinect_Camera::stop_streaming() {
	if (this->streaming) this->multicam_reference->reference->enabledSources -= pow(2, (this->camera_num));
	this->streaming = false;
}

void KinectMultiCamera::Kinect_Camera::send_data(image::ImagePtr image_data) {
	image::ImageInfoPtr imageinfo(new image::ImageInfo());

	imageinfo->height = this->image_height;
	imageinfo->width = this->image_width;
	imageinfo->step = this->image_step;
	image_data->image_info = imageinfo;
	//image_data->image_info->encoding = this->image_encoding;
	this->rrvar_frame_stream->SendPacket(image_data);
}

void KinectMultiCamera::Kinect_Camera::set_frame_stream(PipePtr<image::ImagePtr> value) {
	imaging::Camera_default_impl::set_frame_stream(value);
	this->rrvar_frame_stream->SetMaxBacklog(3);
}

imaging::camerainfo::CameraInfoPtr KinectMultiCamera::Kinect_Camera::get_camera_info()
{
	return (this->camera_info);
}

KinectBodyTracker::KinectBodyTracker(Kinect2_impl* reference) {
	this->reference = reference;
	this->tracking_active = false;
}

KinectBodyTracker::~KinectBodyTracker() {
	this->tracking_active = false;
}

RR_INTRUSIVE_PTR <sensor::SensorInfo> KinectBodyTracker::get_sensor_info() {
	
	RR_INTRUSIVE_PTR<sensor::SensorInfo > sensor_info(new sensor::SensorInfo());
	return sensor_info;
}

void KinectBodyTracker::set_active(RobotRaconteur::rr_bool active) {
	if (this->tracking_active != active.value) {
		this->tracking_active = active.value;
		if (active.value) {
			this->reference->enabledSources += 32;
		}
		if (!active.value) {
			this->reference->enabledSources -= 32;
		}
	}
}
RobotRaconteur::rr_bool KinectBodyTracker::get_active() {
	return this->tracking_active;
}

device::DeviceInfoPtr KinectBodyTracker::get_device_info() {
	return this->device_info;
}

void KinectBodyTracker::set_kinect_body_sensor_data(PipePtr<edu::rpi::robotics::kinect::KinectBodiesSensorDataPtr >  value) {
	edu::rpi::robotics::kinect::Kinect_default_impl::set_kinect_body_sensor_data(value);

	//imaging::Camera_default_impl::set_frame_stream(value);

	this->rrvar_kinect_body_sensor_data->SetMaxBacklog(10);
}



RR_INTRUSIVE_PTR<RobotRaconteur::RRArray<uint64_t > > KinectBodyTracker::getTrackedBodyIDs()
{
	boost::lock_guard<boost::mutex> guard(this->reference->mtx_);

	RR_INTRUSIVE_PTR<RobotRaconteur::RRArray<uint64_t > > ids = RobotRaconteur::AttachRRArrayCopy<uint64_t >(reinterpret_cast<uint64_t *>(&this->reference->tracked_body_ids), 6);

	return ids;

}

void KinectBodyTracker::getDetectedBodyThreaded(k4abt_frame_t body_frame, int num_bodies)
{
	//boost::lock_guard<boost::mutex> guard(mtx_);
	//TODO: Include more consistent tracking of person or include number of people
	//std::vector<RR_INTRUSIVE_PTR<sensors::kinect2::KinectBody>> bodies;
	RR_INTRUSIVE_PTR<edu::rpi::robotics::kinect::KinectBodies > bodies(new edu::rpi::robotics::kinect::KinectBodies());
	bodies->bodies= RR::AllocateEmptyRRList<edu::rpi::robotics::kinect::KinectBody >();
	for (size_t i = 0; i < num_bodies; i++)
	{
		

		k4abt_skeleton_t skeleton;
		k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
		uint32_t id = k4abt_frame_get_body_id(body_frame, i);
		RR_INTRUSIVE_PTR<edu::rpi::robotics::kinect::KinectBody > body(new edu::rpi::robotics::kinect::KinectBody());
		uint8_t body_joint_states[(int)K4ABT_JOINT_COUNT];
		//double body_joint_positions[JointType::JointType_Count * 3];
		RR::RRNamedArrayPtr<geometry::Pose> poses = RR::AllocateEmptyRRNamedArray<geometry::Pose>((int)K4ABT_JOINT_COUNT);
		geometry::Pose* poses_ptr = &poses->at(0);

		//skeleton.joints;
		for (int joint_id = 0; joint_id < (int)K4ABT_JOINT_COUNT; joint_id++, poses_ptr++) {
			//TODO: Change to use Geometryf
			poses_ptr->s.position.s.x = skeleton.joints[joint_id].position.xyz.x;
			poses_ptr->s.position.s.y = skeleton.joints[joint_id].position.xyz.y;
			poses_ptr->s.position.s.z = skeleton.joints[joint_id].position.xyz.z;
			poses_ptr->s.orientation.s.w = skeleton.joints[joint_id].orientation.wxyz.w;
			poses_ptr->s.orientation.s.x = skeleton.joints[joint_id].orientation.wxyz.x;
			poses_ptr->s.orientation.s.y = skeleton.joints[joint_id].orientation.wxyz.y;
			poses_ptr->s.orientation.s.z = skeleton.joints[joint_id].orientation.wxyz.z;
			body_joint_states[joint_id] = skeleton.joints[joint_id].confidence_level;
		}

		uint64_t tracking_id = (uint64_t)id;
		BOOLEAN is_tracked = false;

		body->tracked = (is_tracked) ? 1 : 0;
		body->tracking_id = tracking_id;
		body->joint_poses = poses;
		//body->joint_positions = RobotRaconteur::AttachRRArrayCopy<double >(reinterpret_cast<double *>(&body_joint_positions), JointType::JointType_Count * 3);
		//body->joint_orientations = RobotRaconteur::AttachRRArrayCopy<double >(reinterpret_cast<double *>(&body_joint_orientations), JointType::JointType_Count * 4);
		body->joint_tracking_state = RobotRaconteur::AttachRRArrayCopy<uint8_t >(reinterpret_cast<uint8_t *>(body_joint_states), 32);
		bodies->bodies->push_back(body);
	}
	RR_INTRUSIVE_PTR<edu::rpi::robotics::kinect::KinectBodiesSensorData > bodydata(new edu::rpi::robotics::kinect::KinectBodiesSensorData());
	//bodydata->bodies = bodies;
	bodydata->bodies = bodies;
	this->rrvar_kinect_body_sensor_data->SendPacket(bodydata);
	this->rrvar_kinect_body_sensor_value->SetOutValue(bodies);
	
	//return body;
}

KinectIMU::KinectIMU(Kinect2_impl* reference ) {
	this->seqno = 0;
	this->reference = reference;
	this->imu_active = false;
}

sensor::SensorInfoPtr KinectIMU::get_sensor_info() {
	//sensor::SensorInfoPtr ptr = RR_INTRUSIVE_PTR<sensor::SensorInfoPtr>(this->sensorinfo);
	return this->sensorinfo;
		
}

void KinectIMU::set_active(RobotRaconteur::rr_bool active) {
	//std::cout << "setting point cloud active" << std::endl;
	if (this->imu_active != active.value) {
		this->imu_active = active.value;
		if (active.value) {
			this->reference->enabledSources += 128;
			k4a_device_start_imu(this->reference->kinect);
		}
		if (!active.value) {
			k4a_device_stop_imu(this->reference->kinect);
			this->reference->enabledSources -= 128;
			
		}
	}
}
RobotRaconteur::rr_bool KinectIMU::get_active() {
	return this->imu_active;
}

void KinectIMU::set_sensor_data(PipePtr<sensor::SensorDataPtr> value) {
	this->rrvar_sensor_data = RR_MAKE_SHARED<RobotRaconteur::PipeBroadcaster<sensor::SensorDataPtr> >();
	this->rrvar_sensor_data->Init(value);
	this->rrvar_sensor_data->SetMaxBacklog(3);
}

void KinectIMU::send_data(k4a_imu_sample_t sample){
	sensor::SensorDataPtr sensor_data = new (sensor::SensorData);
	com::robotraconteur::sensordata::SensorDataHeaderPtr sensor_data_header = new (com::robotraconteur::sensordata::SensorDataHeader);
	this->seqno++;
	sensor_data_header->seqno = this->seqno;
	com::robotraconteur::datatype::DataTypePtr data_type = new(com::robotraconteur::datatype::DataType);
	/*data_type->name = "Kinect Azure IMU Data";
	data_type->container_type_code = com::robotraconteur::datatype::ContainerTypeCode::ContainerTypeCode(0);
	data_type->type_code = com::robotraconteur::datatype::DataTypeCode::DataTypeCode(1);
	data_type->array_type_code = com::robotraconteur::datatype::ArrayTypeCode::ArrayTypeCode(1);
	data_type->array_var_len = false;
	UINT32 array_len[1] = { 7 };
	RRArrayPtr<UINT32> array_len_val = AttachRRArrayCopy(array_len, 1);
	data_type->array_len = array_len_val;
	sensor_data->data_type = data_type;
	*/
	sensor_data->data_type = this->sensorinfo->data_type;
	sensor_data->data_header = sensor_data_header;



	com::robotraconteur::imu::ImuStatePtr imu_state = new(com::robotraconteur::imu::ImuState);
	
	double acc_x = (double)(sample.acc_sample.xyz.x);
	double acc_y = (double)(sample.acc_sample.xyz.y);
	double acc_z = (double)(sample.acc_sample.xyz.z);
	double gyro_x = (double)(sample.gyro_sample.xyz.x);
	double gyro_y = (double)(sample.gyro_sample.xyz.y);
	double gyro_z = (double)(sample.gyro_sample.xyz.z);
	double temperature = (double)(sample.temperature);
	double array_data_send[7] = { acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,temperature };
	RRArrayPtr<double> data_send = AttachRRArrayCopy(array_data_send, 7);
	sensor_data->data = data_send;
	this->rrvar_sensor_data->SendPacket(sensor_data);
	this->rrvar_sensor_value->SetOutValue(data_send);
	//sensor_data->data = imudata;
	geometry::Vector3 vector_acc;
	geometry::Vector3 vector_g;
	vector_acc.s.x = acc_x;
	vector_acc.s.y = acc_y;
	vector_acc.s.z = acc_z;
	vector_g.s.x = gyro_x;
	vector_g.s.y = gyro_y;
	vector_g.s.z = gyro_z;
	imu_state->angular_velocity = vector_g;
	imu_state->linear_acceleration = vector_acc;
	this->rrvar_imu_state->SetOutValue(imu_state);

}



Kinect2_impl::Kinect2_impl(int depth_image_height, int depth_image_width, int color_image_height, int color_image_width)
{
	
	YAML::Node config = YAML::LoadFile("KinectSensor.yaml");
	if (config["device_info"]["device1"]["name"] ) {
		std::cout << (config["device_info"]["device1"]["name"]);
	}
	//device::DeviceInfoPtr deviceinfo(new device::DeviceInfo());
	imaging::camerainfo::MultiCameraInfoPtr multicamera_info(new imaging::camerainfo::MultiCameraInfo());
	multicamera_info = config["device_info"]["multicamera"].as<imaging::camerainfo::MultiCameraInfoPtr>();
	
	//std::cout << (multicamera_info->device_info->manufacturer->name);

	pointcloud::sensor::PointCloudSensorInfoPtr pointcloudsensorinfo(new pointcloud::sensor::PointCloudSensorInfo());
	pointcloudsensorinfo = config["device_info"]["point_cloud"].as<pointcloud::sensor::PointCloudSensorInfoPtr>();

	sensor::SensorInfoPtr body_tracker_info(new sensor::SensorInfo);
	body_tracker_info = config["device_info"]["body_tracker"].as<sensor::SensorInfoPtr>();

	sensor::SensorInfoPtr imu_info(new sensor::SensorInfo);
	imu_info = config["device_info"]["imu"].as<sensor::SensorInfoPtr>();

	RR_SHARED_PTR<KinectPointCloud> pointcloud = RR_MAKE_SHARED<KinectPointCloud>(this);
	this->pointcloud = pointcloud;
	this->pointcloud->sensor_infos = pointcloudsensorinfo;

	
	RR_SHARED_PTR<KinectBodyTracker> tracker = RR_MAKE_SHARED<KinectBodyTracker>(this);
	this->tracker = tracker;
	//this->tracker->sensor_info_stored = body_tracker_info;
	RR_SHARED_PTR<KinectMultiCamera> multicamera= RR_MAKE_SHARED<KinectMultiCamera>(this);
	this->multicamera = multicamera;
	//this->multicamera->multicamera_info = multicamera_info;
	//std::list< imaging::camerainfo::CameraInfoPtr  >::iterator current_val = this->multicamera->multicamera_info->camera_info_all->begin();

	//this->multicamera->cameras[0]->camera_info = *current_val;
	//std::next(current_val, 1);

	//this->multicamera->cameras[1]->camera_info = *current_val;
	//std::next(current_val, 1);
	//this->multicamera->cameras[2]->camera_info = *current_val;
	//std::next(current_val, 1);
	//this->multicamera->cameras[3]->camera_info = *current_val;

	RR_SHARED_PTR<KinectIMU> imu = RR_MAKE_SHARED<KinectIMU>(this);
	this->imu = imu;
	//this->imu->sensorinfo = imu_info;

	//this

	this->thread_exit = false;
	TIMEOUT_IN_MS = 100;
	//this->enabledSources = FrameSourceTypes_None;
	this->color_image_width = color_image_width;
	this->color_image_height = color_image_height;
	//std::cout << "Success" << std::endl;
	this->depth_image_width = depth_image_width;
	this->depth_image_height = depth_image_height;
	
	// Allocate memory for the different image streams (consider moving this to the enable_streams section?)
	
	this->bodyindex_image_data = new uint8_t[depth_image_width * depth_image_height];
	this->depth_image_data = new uint16_t[depth_image_width * depth_image_height];
	this->infrared_image_data = new uint16_t[depth_image_width * depth_image_height];
	//this->longexposure_infrared_image_data = new uint16_t[this->depth_image_width * this->depth_image_height];
	this->color_image_data = new uint8_t[color_image_width * color_image_height];
	//this->pointcloud_temp = new uint16_t[this->depth_image_width*this->depth_image_height * 6];
	this->infrared_image_size, this->color_image_size, this->body_index_size, this->depth_image_size = 0;
	//KinectPointCloud pointcloud (new KinectPointCloud(this));
	//this->pointcloud = KinectPointCloud(this);
	HRESULT hr = StartupKinect();
	if (FAILED(hr))
	{
		std::cout << "Failed to Startup Kinect: error code " << HRESULT_CODE(hr) << std::endl;
		return;
	}
	
}

Kinect2_impl::~Kinect2_impl()
{
	ShutdownKinect();
	delete bodyindex_image_data;
	delete depth_image_data;
	delete infrared_image_data;
	//delete longexposure_infrared_image_data;
	delete color_image_data;
}

HRESULT Kinect2_impl::StartupKinect()
{
	HRESULT hr = NULL;

	// Attempt access to default Kinect-2 sensor
	std::cout << "Looking for Default Kinect Sensor" << std::endl;
	//k4a_device  k4a_device_open(&this->kinect);
	uint32_t device_count = k4a_device_get_installed_count();
	if (device_count < 1) {
		printf("No Kinect found");
		return hr;
	}
	k4a_device_t kinect = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &kinect)) {
		printf("Failed to connect to default kinect");
	}
	this->kinect = kinect;
	// If Kinect is found, initialize
	if (this->kinect)
	{
		std::cout << "Found Kinect, Initializing..." << std::endl;

		//hr = this->kinect->Open();
		//if FAILED(hr) { return hr; }
		this->enabledSources = 0;
		// Get Color Frame Information
		std::cout << "Getting Color Frame Info...";
		
		std::cout << "Success" << std::endl;

		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config.camera_fps = K4A_FRAMES_PER_SECOND_30;
		config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		k4a_calibration_t sensor_calibration;
		if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(kinect, config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration))
		{
			printf("Get depth camera calibration failed!\n");
			return 0;
		}

		config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
		switch (this->color_image_height) {
		case 720:
			config.color_resolution= K4A_COLOR_RESOLUTION_720P;
			break;
		case 1080:
			config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
			break;
		case 1440:
			config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
			break;
		case 1536:
			config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
			break;
		case 2160:
			config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
			break;
		case 3072:
			config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
			break;
		}
		switch (this->depth_image_height) {
		case 288:
			config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
			break;
		case 576:
			config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
			break;
		case 512:
			config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
			break;
		case 1024:
			config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
			break;
		}
		//config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
		//this->enabledSources++;
		
		
		k4a_image_t pointcloudimagetemp = NULL;
		k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, this->depth_image_width, this->depth_image_height, this->depth_image_width*(int)sizeof(PointCloudPixel_int16x3_t), &pointcloudimagetemp);
		this->pointcloudimagetemp = pointcloudimagetemp;

		//config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		transform = k4a_transformation_create(&sensor_calibration);
			
		k4abt_tracker_t kinect_body_tracker = NULL;
		k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
		if (K4A_RESULT_SUCCEEDED != k4abt_tracker_create(&sensor_calibration, tracker_config, &kinect_body_tracker))
		{
			printf("Body tracker initialization failed!\n");
			return 0;
		}
		this->kinect_body_tracker = kinect_body_tracker;
		

		
		this->config = config;
		k4a_device_start_cameras(kinect, &config);
		this->thread_exit = false;
		t1 = boost::thread(boost::bind(&Kinect2_impl::backgroundPollingThread, this));
		std::cout << "Success" << std::endl;
		
		
	}

	return hr;
}

HRESULT Kinect2_impl::ShutdownKinect()
{
	HRESULT hr = E_FAIL;

	this->enabledSources = 0;
	
	t1.interrupt();
	t1.join();

	if (this->kinect)
		k4a_device_close(this->kinect);
	
	return hr;
}


imaging::CameraPtr KinectMultiCamera::get_cameras(int32_t ind)
{
	return this->cameras.at(ind);
}

imaging::camerainfo::MultiCameraInfoPtr KinectMultiCamera::get_multicamera_info()
{
	return this->multicamera_info;
}


void Kinect2_impl::CaptureArrived(k4a_capture_t* capture)
{
	if (this->enabledSources & 128) {
		k4a_imu_sample_t imu_sample;
		k4a_device_get_imu_sample(this->kinect, &imu_sample, TIMEOUT_IN_MS);

		this->imu->send_data(imu_sample);
	}

	if (this->enabledSources&(1<<5)|| this->enabledSources&(1<<3))
	{
		k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(this->kinect_body_tracker, *capture, 0);
		//std::cout << "starting body track" << std::endl;
		if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
		{
			printf("Error! Adding capture to tracker process queue failed!\n");
			
		}

		k4abt_frame_t body_frame = NULL;
		k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(kinect_body_tracker, &body_frame, 0);
		if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			if (this->enabledSources&(1 << 3)) {
				k4a_image_t body_index = k4abt_frame_get_body_index_map(body_frame);
				image::ImagePtr imagedata(new image::Image());
				//this->mtx_.lock();
				boost::mutex::scoped_lock lock(this->mtx_);
				this->body_index_size = k4a_image_get_size(body_index);
				memcpy(this->bodyindex_image_data, (uint8_t*)(void*)k4a_image_get_buffer(body_index), this->body_index_size);
				
				imagedata->data= AttachRRArrayCopy((uint8_t*)(void*)k4a_image_get_buffer(body_index), this->body_index_size);
				//this->mtx_.unlock();
				this->multicamera->cameras[3]->send_data(imagedata);
				//bodyindex_image_data = k4a_image_get_buffer(body_index);
				k4a_image_release(body_index);
			}
			if (this->enabledSources&(1 << 5)) {
				//this->mtx_.lock();
				boost::mutex::scoped_lock lock(this->mtx_);
				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
				
				this->tracker->getDetectedBodyThreaded(body_frame,num_bodies);

				//for (size_t i = 0; i < num_bodies; i++)
				//{
				//	auto body=this->tracker->getDetectedBodyThreaded(body_frame, i);
				//}
				//this->mtx_.unlock();
				
			}
			k4abt_frame_release(body_frame);
		}
	}
	
	if (this->enabledSources&1 || this->enabledSources&64)
	{
		//std::cout << "Looking at Color Frame Data" << std::endl;
		//IColorFrameReference *color_frame_reference = NULL;
		// IColorFrame *color_frame = NULL;
		//hr = multi_frame->get_ColorFrameReference(&color_frame_reference);
		// Acquire Frame
		
		//if SUCCEEDED(hr)
		//	hr = color_frame_reference->AcquireFrame(&color_frame);
		k4a_image_t color_frame = k4a_capture_get_color_image(*capture);

		if(color_frame)
		{
			if (this->enabledSources & 1) {
				image::ImagePtr imagedata(new image::Image());

				//std::cout << "Copying to buffer...";
				//this->mtx_.lock();
				boost::mutex::scoped_lock lock(this->mtx_);
				//color_image_data = k4a_image_get_buffer(color_frame);
				this->color_image_size = k4a_image_get_size(color_frame);
				memcpy(this->color_image_data, (uint8_t*)(void*)k4a_image_get_buffer(color_frame), this->color_image_size);
				//std::cout << this->color_image_size << std::endl;
				imagedata->data = AttachRRArrayCopy(this->color_image_data, this->color_image_size);
				//this->mtx_.unlock();
				this->multicamera->cameras[0]->send_data(imagedata);
			}
			
			//std::cout << ((SUCCEEDED(hr)) ? "Success" : "Failed") << std::endl;
		}

		//SafeRelease(color_frame);
		k4a_image_release(color_frame);

	}
	if (this->enabledSources&4||this->enabledSources&16)
	{
		//std::cout << "Looking at Depth Frame Data" << std::endl;
		k4a_image_t depth_frame = k4a_capture_get_depth_image(*capture);
		
		if (depth_frame) {
			if (this->enabledSources & 4)
			{
			
				//std::cout << "Copying to buffer...";
				//UINT buffer_size = k4a_image_get_size(color_frame);
				image::ImagePtr imagedata(new image::Image());

				//std::cout << "Copying to buffer...";
				//this->mtx_.lock();
				boost::mutex::scoped_lock lock(this->mtx_);
				this->depth_image_size = k4a_image_get_size(depth_frame);
				memcpy(this->depth_image_data, (uint16_t*)(void*)k4a_image_get_buffer(depth_frame), this->depth_image_size);
				

				imagedata->data = AttachRRArrayCopy((uint8_t *)this->depth_image_data, this->depth_image_size);
				
				//this->mtx_.unlock();
				this->multicamera->cameras[2]->image_height = k4a_image_get_height_pixels(depth_frame);
				this->multicamera->cameras[2]->image_width = k4a_image_get_width_pixels(depth_frame);
				this->multicamera->cameras[2]->send_data(imagedata);

			}
			if (this->enabledSources & 16) {
				//k4a_image_t pointcloudimagetemp = NULL;
				 //points(new sensors::kinect2::PointCloud());
				boost::mutex::scoped_lock lock(this->mtx_);
				k4a_transformation_depth_image_to_point_cloud(this->transform, depth_frame, K4A_CALIBRATION_TYPE_DEPTH, this->pointcloudimagetemp);
				int depth_image_size = k4a_image_get_size(this->pointcloudimagetemp);
				int width = k4a_image_get_width_pixels(this->pointcloudimagetemp);
				int height = k4a_image_get_height_pixels(this->pointcloudimagetemp);
				//const auto pointCloudImageBufferInMM = (PointCloudPixel_int16x3_t*)k4a_image_get_buffer(this->pointcloudimagetemp);
				//uint16_t *pointcloud_temp = (uint16_t*)(void*)k4a_image_get_buffer(pointcloudimagetemp);
				//memcpy(this->pointcloud_temp, (uint16_t*)(void*)k4a_image_get_buffer(this->pointcloudimagetemp), k4a_image_get_size(this->pointcloudimagetemp));
				
				
				pointcloud::sensor::PointCloudSensorDataPtr points(new pointcloud::sensor::PointCloudSensorData());
				points=	this->pointcloud->getPointCloudThreaded(height,width, (PointCloudPixel_int16x3_t*)k4a_image_get_buffer(this->pointcloudimagetemp));
				//this->pointcloud->send_data(points);
				
				//k4a_image_release(pointcloudimagetemp);
			}
			
		}

		
		
		k4a_image_release(depth_frame);
		
		

	}
	if (this->enabledSources&2)
	{
		k4a_image_t ir_frame = k4a_capture_get_ir_image(*capture);
		//std::cout << "Copying to buffer...";

		if (ir_frame)
		{
			
			image::ImagePtr imagedata(new image::Image());

			//std::cout << "Copying to buffer...";
			//this->mtx_.lock();
			boost::mutex::scoped_lock lock(this->mtx_);
			this->infrared_image_size = k4a_image_get_size(ir_frame);
			memcpy(this->infrared_image_data, (uint16_t*)(void*)k4a_image_get_buffer(ir_frame), this->infrared_image_size);
			
			imagedata->data = AttachRRArrayCopy((uint8_t *)this->infrared_image_data, this->infrared_image_size);
			//this->mtx_.unlock();
			this->multicamera->cameras[1]->image_height = k4a_image_get_height_pixels(ir_frame);
			this->multicamera->cameras[1]->image_width = k4a_image_get_width_pixels(ir_frame);
			this->multicamera->cameras[1]->send_data(imagedata);
			
		}
		k4a_image_release(ir_frame);

	}
	
	//k4a_capture_release(*capture);
	
}

void Kinect2_impl::backgroundPollingThread()
{
	//HRESULT hr;
	//DWORD res;
	std::cout << "Starting up background thread" << std::endl;
	while (!(this->thread_exit))
	{
		try
		{
			
			//res = WaitForSingleObjectEx(reinterpret_cast<HANDLE>(h_event), 1000, false);
			k4a_capture_t capture;
			//auto start = chrono::steady_clock::now();
			switch (k4a_device_get_capture(kinect , &capture, TIMEOUT_IN_MS))
			{
			case K4A_WAIT_RESULT_SUCCEEDED:
				//mtx_.lock();
				//printf("Capture obtained\n");
				
				CaptureArrived(&capture);
				k4a_capture_release(capture);
				
				//mtx_.unlock();
				break;
			case K4A_WAIT_RESULT_TIMEOUT:
				printf("Timed out waiting for a capture\n");
				continue;
				break;
			case K4A_WAIT_RESULT_FAILED:
				printf("Failed to read a capture\n");
				break;
			}
			//auto end = chrono::steady_clock::now();
			//std::cout << chrono::duration_cast<chrono::microseconds>(end - start).count() << std::endl;

		}
		catch (...) {
			break;
		}
	}

	std::cout << "Exiting Background Thread" << std::endl;
}

Kinect_impl_playback::Kinect_impl_playback(char* filename)
{
	
	//RR_SHARED_PTR<KinectPointCloud> pointcloud = RR_MAKE_SHARED<KinectPointCloud>(this);
	this->pointcloud = pointcloud;
	this->pointcloud->sensor_infos = nullptr;


	
	//RR_SHARED_PTR<KinectMultiCamera> multicamera = RR_MAKE_SHARED<KinectMultiCamera>(this);
	this->multicamera = multicamera;
	this->multicamera->multicamera_info = nullptr;
	std::list< imaging::camerainfo::CameraInfoPtr  >::iterator current_val = this->multicamera->multicamera_info->camera_info_all->begin();

	this->multicamera->cameras[0]->camera_info = nullptr;
	std::next(current_val, 1);

	this->multicamera->cameras[1]->camera_info = nullptr;
	std::next(current_val, 1);
	this->multicamera->cameras[2]->camera_info = nullptr;
	std::next(current_val, 1);
	this->multicamera->cameras[3]->camera_info = nullptr;

	//RR_SHARED_PTR<KinectIMU> imu = RR_MAKE_SHARED<KinectIMU>(this);
	this->imu = imu;
	this->imu->sensorinfo = nullptr;

	//this

	this->thread_exit = false;
	TIMEOUT_IN_MS = 100;
	//this->enabledSources = FrameSourceTypes_None;
	this->color_image_width = 3840;
	this->color_image_height = 2160;
	//std::cout << "Success" << std::endl;
	this->depth_image_width = 640;
	this->depth_image_height = 576;

	// Allocate memory for the different image streams (consider moving this to the enable_streams section?)

	this->bodyindex_image_data = new uint8_t[this->depth_image_width * this->depth_image_height];
	this->depth_image_data = new uint16_t[this->depth_image_width * this->depth_image_height];
	this->infrared_image_data = new uint16_t[this->depth_image_width * this->depth_image_height];
	//this->longexposure_infrared_image_data = new uint16_t[this->depth_image_width * this->depth_image_height];
	this->color_image_data = new uint8_t[this->color_image_width * this->color_image_height];
	//this->pointcloud_temp = new uint16_t[this->depth_image_width*this->depth_image_height * 6];
	this->infrared_image_size, this->color_image_size, this->body_index_size, this->depth_image_size = 0;
	//KinectPointCloud pointcloud (new KinectPointCloud(this));
	//this->pointcloud = KinectPointCloud(this);
	HRESULT hr = StartupKinect();
	if (FAILED(hr))
	{
		std::cout << "Failed to Startup Kinect: error code " << HRESULT_CODE(hr) << std::endl;
		return;
	}



	
}

Kinect_impl_playback::~Kinect_impl_playback()
{

}

HRESULT Kinect_impl_playback::StartupKinect()
{
	HRESULT hr = NULL;

	// Attempt access to default Kinect-2 sensor
	std::cout << "Looking for Default Kinect Sensor" << std::endl;
	//k4a_device  k4a_device_open(&this->kinect);
	uint32_t device_count = k4a_device_get_installed_count();
	if (device_count < 1) {
		printf("No Kinect found");
		return hr;
	}
	k4a_device_t kinect = NULL;
	if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &kinect)) {
		printf("Failed to connect to default kinect");
	}
	this->kinect = kinect;
	// If Kinect is found, initialize
	if (this->kinect)
	{
		std::cout << "Found Kinect, Initializing..." << std::endl;

		//hr = this->kinect->Open();
		//if FAILED(hr) { return hr; }
		
		



		
		
		this->thread_exit = false;
		t1 = boost::thread(boost::bind(&Kinect_impl_playback::backgroundPollingThread, this));
		std::cout << "Success" << std::endl;


	}

	return hr;
	return E_NOTIMPL;
}



void Kinect_impl_playback::CaptureArrived(k4a_capture_t* capture)
{

	if (k4a_playback_get_next_imu_sample(this->playback, &this->imu_sample) == K4A_STREAM_RESULT_SUCCEEDED) {
		this->imu->send_data(this->imu_sample);
	}
	
	/*
	if (this->enabledSources & (1 << 5) || this->enabledSources & (1 << 3))
	{
		k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(this->kinect_body_tracker, *capture, 0);
		//std::cout << "starting body track" << std::endl;
		if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
		{
			printf("Error! Adding capture to tracker process queue failed!\n");

		}

		k4abt_frame_t body_frame = NULL;
		k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(kinect_body_tracker, &body_frame, 0);
		if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			if (this->enabledSources & (1 << 3)) {
				k4a_image_t body_index = k4abt_frame_get_body_index_map(body_frame);
				image::ImagePtr imagedata(new image::Image());
				//this->mtx_.lock();
				boost::mutex::scoped_lock lock(this->mtx_);
				this->body_index_size = k4a_image_get_size(body_index);
				memcpy(this->bodyindex_image_data, (uint8_t*)(void*)k4a_image_get_buffer(body_index), this->body_index_size);

				imagedata->data = AttachRRArrayCopy((uint8_t*)(void*)k4a_image_get_buffer(body_index), this->body_index_size);
				//this->mtx_.unlock();
				this->multicamera->cameras[3]->send_data(imagedata);
				//bodyindex_image_data = k4a_image_get_buffer(body_index);
				k4a_image_release(body_index);
			}
			if (this->enabledSources & (1 << 5)) {
				//this->mtx_.lock();
				boost::mutex::scoped_lock lock(this->mtx_);
				size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);

				this->tracker->getDetectedBodyThreaded(body_frame, num_bodies);

				//for (size_t i = 0; i < num_bodies; i++)
				//{
				//	auto body=this->tracker->getDetectedBodyThreaded(body_frame, i);
				//}
				//this->mtx_.unlock();

			}
			k4abt_frame_release(body_frame);
		}
	}
	*/
	k4a_image_t color_frame = k4a_capture_get_color_image(*capture);
	if (color_frame)
	{
		//std::cout << "Looking at Color Frame Data" << std::endl;
		//IColorFrameReference *color_frame_reference = NULL;
		// IColorFrame *color_frame = NULL;
		//hr = multi_frame->get_ColorFrameReference(&color_frame_reference);
		// Acquire Frame

		//if SUCCEEDED(hr)
		//	hr = color_frame_reference->AcquireFrame(&color_frame);
		//k4a_image_t color_frame = k4a_capture_get_color_image(*capture);

		
		image::ImagePtr imagedata(new image::Image());

		//std::cout << "Copying to buffer...";
		//this->mtx_.lock();
		boost::mutex::scoped_lock lock(this->mtx_);
		//color_image_data = k4a_image_get_buffer(color_frame);
		this->color_image_size = k4a_image_get_size(color_frame);
		memcpy(this->color_image_data, (uint8_t*)(void*)k4a_image_get_buffer(color_frame), this->color_image_size);
		//std::cout << this->color_image_size << std::endl;
		imagedata->data = AttachRRArrayCopy(this->color_image_data, this->color_image_size);
		//this->mtx_.unlock();
		this->multicamera->cameras[0]->send_data(imagedata);
			

		//SafeRelease(color_frame);
		k4a_image_release(color_frame);

	}
	k4a_image_t depth_frame = k4a_capture_get_depth_image(*capture);
	
		//std::cout << "Looking at Depth Frame Data" << std::endl;
		

	if (depth_frame) {
		

		//std::cout << "Copying to buffer...";
		//UINT buffer_size = k4a_image_get_size(color_frame);
		image::ImagePtr imagedata(new image::Image());

		//std::cout << "Copying to buffer...";
		//this->mtx_.lock();
		boost::mutex::scoped_lock lock(this->mtx_);
		this->depth_image_size = k4a_image_get_size(depth_frame);
		memcpy(this->depth_image_data, (uint16_t*)(void*)k4a_image_get_buffer(depth_frame), this->depth_image_size);


		imagedata->data = AttachRRArrayCopy((uint8_t*)this->depth_image_data, this->depth_image_size);

		//this->mtx_.unlock();
		this->multicamera->cameras[2]->image_height = k4a_image_get_height_pixels(depth_frame);
		this->multicamera->cameras[2]->image_width = k4a_image_get_width_pixels(depth_frame);
		this->multicamera->cameras[2]->send_data(imagedata);

		
		
		//k4a_image_t pointcloudimagetemp = NULL;
			//points(new sensors::kinect2::PointCloud());
		
		k4a_transformation_depth_image_to_point_cloud(this->transform, depth_frame, K4A_CALIBRATION_TYPE_DEPTH, this->pointcloudimagetemp);
		int depth_image_size = k4a_image_get_size(this->pointcloudimagetemp);
		int width = k4a_image_get_width_pixels(this->pointcloudimagetemp);
		int height = k4a_image_get_height_pixels(this->pointcloudimagetemp);
		//const auto pointCloudImageBufferInMM = (PointCloudPixel_int16x3_t*)k4a_image_get_buffer(this->pointcloudimagetemp);
		//uint16_t *pointcloud_temp = (uint16_t*)(void*)k4a_image_get_buffer(pointcloudimagetemp);
		//memcpy(this->pointcloud_temp, (uint16_t*)(void*)k4a_image_get_buffer(this->pointcloudimagetemp), k4a_image_get_size(this->pointcloudimagetemp));


		pointcloud::sensor::PointCloudSensorDataPtr points(new pointcloud::sensor::PointCloudSensorData());
		points = this->pointcloud->getPointCloudThreaded(height, width, (PointCloudPixel_int16x3_t*)k4a_image_get_buffer(this->pointcloudimagetemp));
		//this->pointcloud->send_data(points);

		//k4a_image_release(pointcloudimagetemp);
			

		



		k4a_image_release(depth_frame);



	}
	k4a_image_t ir_frame = k4a_capture_get_ir_image(*capture);
	
	if (ir_frame)
	{

		image::ImagePtr imagedata(new image::Image());

		//std::cout << "Copying to buffer...";
		//this->mtx_.lock();
		boost::mutex::scoped_lock lock(this->mtx_);
		this->infrared_image_size = k4a_image_get_size(ir_frame);
		memcpy(this->infrared_image_data, (uint16_t*)(void*)k4a_image_get_buffer(ir_frame), this->infrared_image_size);

		imagedata->data = AttachRRArrayCopy((uint8_t*)this->infrared_image_data, this->infrared_image_size);
		//this->mtx_.unlock();
		this->multicamera->cameras[1]->image_height = k4a_image_get_height_pixels(ir_frame);
		this->multicamera->cameras[1]->image_width = k4a_image_get_width_pixels(ir_frame);
		this->multicamera->cameras[1]->send_data(imagedata);

		
		k4a_image_release(ir_frame);

	}

	//k4a_capture_release(*capture);

}

void Kinect_impl_playback::backgroundPollingThread()
{
	//HRESULT hr;
	//DWORD res;
	std::cout << "Starting up background thread" << std::endl;
	while (!(this->thread_exit))
	{
		try
		{

			//res = WaitForSingleObjectEx(reinterpret_cast<HANDLE>(h_event), 1000, false);
			k4a_capture_t capture;
			
			
			//auto start = chrono::steady_clock::now();
			switch (k4a_playback_get_next_capture(this->playback, &capture))
			{
			case K4A_WAIT_RESULT_SUCCEEDED:
				//mtx_.lock();
				//printf("Capture obtained\n");

				CaptureArrived(&capture);
				k4a_capture_release(capture);

				//mtx_.unlock();
				break;
			
			case K4A_STREAM_RESULT_FAILED:
				printf("Failed to read a capture\n");
				break;
			case K4A_STREAM_RESULT_EOF:
				printf("End of file reached\n");
				break;
				k4a_playback_close(this->playback);
				this->thread_exit = true;
			}
			//auto end = chrono::steady_clock::now();
			//std::cout << chrono::duration_cast<chrono::microseconds>(end - start).count() << std::endl;

		}
		catch (...) {
			break;
		}
	}

	std::cout << "Exiting Background Thread" << std::endl;
}
