
#include "kinect2_impl.h"

int main(int argc, char* argv[])
{

	std::string dummy;
	if (argc < 3) {
		std::cout << "If using for playback, Usage: " << argv[0] << "--load PLAYBACK FILE " << std::endl;
	}
	if (argc == 3) {

	}
	RR_SHARED_PTR<Kinect2_impl> k = RR_MAKE_SHARED<Kinect2_impl>();
	
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