
#include "kinect2_impl.h"
namespace po = boost::program_options;
int main(int argc, char* argv[])
{

	std::string dummy;
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("depth_height", po::value<int>()->default_value(576), "set depth camera image height")
		("depth_width", po::value<int>()->default_value(640), "set depth camera image width")
		("color_width", po::value<int>()->default_value(3840), "set color camera image width")
		("color_height", po::value<int>()->default_value(2160), "set color camera image height")
		("playback_mode", po::value<bool>()->default_value(false), "set for playback mode")
		("playback_file", po::value<std::string>()->default_value(""), "file for playback mode");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);
	int depth_height = 576;
	int depth_width =640;
	int color_image_width = 3840;
	int color_image_height = 2160;
	
	if (vm.count("help")) {
		std::cout << desc << "\n";
		return 0;
	}
	std::map<int, int> color_values = { {1280, 720}, {1920,1080 },
									{2560, 1440}, {2048,1536 },{3840,2160},{4096,3072} };
	std::map<int, int> depth_values = { {320, 288}, {512,512 },
									{640, 576}, {1024,1024 } };
	if (vm.count("depth_height") && vm.count("depth_width")) {
		
		if (depth_values.count(vm["depth_width"].as<int>())>0) {
			if (depth_values.at(vm["depth_width"].as<int>()) == vm["depth_height"].as<int>()) {
				depth_height =vm["depth_height"].as<int>();
				depth_width = vm["depth_width"].as<int>();
			}
			else {
				std::cerr << "Incorrect value provided for depth camera width, must match depth camera height " << std::endl;
				return 0;
			}
		}
		else {
			std::cerr << "Incorrect resolution values provided for depth camera" << std::endl;
			return 0;
		}
		
	}
	if (vm.count("color_height") && vm.count("color_width")) {
		if (color_values.count(vm["color_width"].as<int>())>0) {
			if (color_values.at(vm["color_width"].as<int>()) == vm["color_height"].as<int>()) {
				color_image_height = vm["color_height"].as<int>();
				color_image_width = vm["color_width"].as<int>();
			}
			else {
				std::cerr << "Incorrect value provided for color camera width, must match color camera height " << std::endl;
				return 0;
			}
		}
		else {
			std::cerr << "Incorrect resolution values provided for color camera" << std::endl;
			return 0;
		}

	}
	
	std::cout << depth_height;
	std::cout << depth_width;
	RR_SHARED_PTR<Kinect2_impl> k = RR_MAKE_SHARED<Kinect2_impl>(depth_height,depth_width,color_image_height,color_image_width);
	
	RobotRaconteur::Companion::RegisterStdRobDefServiceTypes();
	//RR::ServerNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES, "edu.rpi.robotics.kinect", 8888);
	RR::ServerNodeSetup node_setup(std::vector<RR::ServiceFactoryPtr>(), "edu.rpi.robotics.kinect", 8888);
	/*
	// Register Local Transport
	boost::shared_ptr<RobotRaconteur::LocalTransport> t1 = boost::make_shared<RobotRaconteur::LocalTransport>();
	t1->StartServerAsNodeName("sensors.kinect2");
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t1);

	// Register TCP Transport on port 8888
	boost::shared_ptr<RobotRaconteur::TcpTransport> t = boost::make_shared<RobotRaconteur::TcpTransport>();
	t->StartServer(8888)w
	t->EnableNodeAnnounce(	RobotRaconteur::IPNodeDiscoveryFlags_LINK_LOCAL |
							RobotRaconteur::IPNodeDiscoveryFlags_NODE_LOCAL |
							RobotRaconteur::IPNodeDiscoveryFlags_SITE_LOCAL);
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t);

	// Create the Kinect object
	

	// Register the service type with Robot Raconteur
	RobotRaconteur::RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<sensors::kinect2::sensors__kinect2Factory>());
*/
	// Register the Kinect object as a service
	
	RobotRaconteur::RobotRaconteurNode::s()->RegisterServiceType(RR_MAKE_SHARED<edu::rpi::robotics::kinect::edu__rpi__robotics__kinectFactory>());
	RobotRaconteur::RobotRaconteurNode::s()->RegisterService("KinectTracker", "edu.rpi.robotics.kinect", k->tracker);
	RobotRaconteur::RobotRaconteurNode::s()->RegisterService("KinectPointCloud", "com.robotraconteur.pointcloud.sensor", k->pointcloud);
	RobotRaconteur::RobotRaconteurNode::s()->RegisterService("KinectMultiCamera", "com.robotraconteur.imaging", k->multicamera);
	RobotRaconteur::RobotRaconteurNode::s()->RegisterService("KinectIMU", "com.robotraconteur.imu", k->imu);
	std::cout << "Connect to the Kinect v2 Service at: " << std::endl;
	std::cout << "tcp://localhost:8888/sensors.kinect2/Kinect2" << std::endl;
	std::cout << "Press enter to finish" << std::endl;
	std::getline(std::cin, dummy);
	k->ShutdownKinect();
	return 0;
}