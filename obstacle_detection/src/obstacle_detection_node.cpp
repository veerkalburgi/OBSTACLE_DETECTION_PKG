#include "obstacle_detection/obstacle_detection_node.h"

//Debug defines
#define PUB_DEBUG 0

ObstacleDetection::ObstacleDetection()
{
	nh = new ros::NodeHandle();
	vehicle_polygon_pub = nh->advertise<geometry_msgs::PolygonStamped>("/vehicle_polygon", 1);
	warning_zone_polygon_pub = nh->advertise<geometry_msgs::PolygonStamped>("/warning_zone_polygon", 1);
	stop_zone_polygon_pub = nh->advertise<geometry_msgs::PolygonStamped>("/stop_zone_polygon", 1);
	camera_polygon_pub = nh->advertise<geometry_msgs::PolygonStamped>("/camera_polygon", 1);
	obstacle_zone_pub = nh->advertise<std_msgs::Int16>("/warning_zone_fused", 1);
	current_warning_zone_pub = nh->advertise<std_msgs::Int8>("/current_warning_zone", 1);
	current_stop_zone_pub = nh->advertise<std_msgs::Int8>("/current_stop_zone", 1);

	listener = new tf::TransformListener;

	zone_select_server_instance = nh->advertiseService("set_obstacle_zones", &ObstacleDetection::SetObstacleZonesCallback, this);

	last_obstacle_trigger = ros::Time::now();

#if PUB_DEBUG
	pcl_pub_wo_veh = nh->advertise<sensor_msgs::PointCloud2>("/agg_without_vehicle", 1);
	pcl_pub_warning = nh->advertise<sensor_msgs::PointCloud2>("/agg_warning", 1);
	pcl_pub_stop = nh->advertise<sensor_msgs::PointCloud2>("/agg_stop", 1);
	pcl_pub_agg = nh->advertise<sensor_msgs::PointCloud2>("/agg_cloud", 1);
	pcl_pub_laser = nh->advertise<sensor_msgs::PointCloud2>("/laser_cloud_viz", 1);
	pcl_camera = nh->advertise<sensor_msgs::PointCloud2>("/front_camer_box_filter_viz", 1);
#endif

	minimum_obstacle_size = 0.01;
	total_warning_zones = 2;
	total_stop_zones = 2;
	obstacle_persistence = 1.0;
	use_laser = true;
	use_front_camera = true;
	use_back_camera = false;
	use_ultrasonic = false;

	//other variables
	warning_zone_id = 0;
	stop_zone_id = 0;
	transforms_computed = false;
	transform_okay = true;
	vehicle_frame_id = "/base_link";
	empty_frame_id = "/base_link";
}


void ObstacleDetection::load_params()
{
	std::string str_;
	char char_[50];
	bool is_stop_zone_valid_ = true, is_warning_zone_valid_ = true;
	bool zone_okay, params_loaded;

	params_loaded = nh->getParam("/minimum_obstacle_size", minimum_obstacle_size);

	params_loaded = params_loaded && nh->getParam("/observation_sources/laser/enable", use_laser);
	nh->param<std::string>("/observation_sources/laser/topic", laser_sub_topic, laser_sub_topic);

	params_loaded = params_loaded && nh->getParam("/observation_sources/ultrasonic/enable", use_ultrasonic);
	nh->param<std::string>("/observation_sources/ultrasonic/topic", ultrasonic_sub_topic, ultrasonic_sub_topic);

	params_loaded = params_loaded && nh->getParam("/observation_sources/front_camera/enable", use_front_camera);
	nh->param<std::string>("/observation_sources/front_camera/topic", front_camera_sub_topic, front_camera_sub_topic);

	params_loaded = params_loaded && nh->getParam("/observation_sources/back_camera/enable", use_back_camera);
	nh->param<std::string>("/observation_sources/back_camera/topic", back_camera_sub_topic, back_camera_sub_topic);

	params_loaded = params_loaded && nh->getParam("/vehicle_footprint", str_);
	sscanf(str_.c_str(), "[%lf, %lf, %lf, %lf, %lf, %lf]", &vehicle.minx, &vehicle.miny, &vehicle.minz, &vehicle.maxx,
			&vehicle.maxy, &vehicle.maxz);
	str_.clear();
	params_loaded = params_loaded && nh->getParam("/camera_box", str_);
	sscanf(str_.c_str(), "[%lf, %lf, %lf, %lf, %lf, %lf]", &camera.minx, &camera.miny, &camera.minz, &camera.maxx,
			&camera.maxy, &camera.maxz);
	str_.clear();

	params_loaded = params_loaded && nh->getParam("/number_of_stop_zones", total_stop_zones);
	stop.resize(total_stop_zones);

	for (int i = 0; i < total_stop_zones; i++)
	{
		sprintf(char_, "/stop_zone%d", i);
		nh->getParam(char_, str_);
		sscanf(str_.c_str(), "[%lf, %lf, %lf, %lf, %lf, %lf]", &stop[i].minx, &stop[i].miny, &stop[i].minz, &stop[i].maxx,
				&stop[i].maxy, &stop[i].maxz);
		str_.clear();

		is_stop_zone_valid_ = ValidateSafetyZone(stop[i], vehicle) && is_stop_zone_valid_;
	}

	params_loaded = params_loaded && nh->getParam("/number_of_warning_zones", total_warning_zones);
	warning.resize(total_warning_zones);

	for (int i = 0; i < total_warning_zones; i++)
	{
		sprintf(char_, "/warning_zone%d", i);
		nh->getParam(char_, str_);
		sscanf(str_.c_str(), "[%lf, %lf, %lf, %lf, %lf, %lf]", &warning[i].minx, &warning[i].miny, &warning[i].minz,
				&warning[i].maxx, &warning[i].maxy, &warning[i].maxz);
		str_.clear();
		is_warning_zone_valid_ = ValidateSafetyZone(warning[i], stop[i]) && is_warning_zone_valid_;
	}

	params_loaded = params_loaded && nh->getParam("/obstacle_persistence", obstacle_persistence);

	ROS_INFO_ONCE ("Loaded Topics: %s, %s, %s, %s", laser_sub_topic.c_str(), ultrasonic_sub_topic.c_str(), front_camera_sub_topic.c_str(), back_camera_sub_topic.c_str());


	zone_okay = is_stop_zone_valid_ && is_warning_zone_valid_;

	if(!is_stop_zone_valid_)
		ROS_WARN("Stop zone not vaild");
	if(!is_warning_zone_valid_)
		ROS_WARN("Warning zone not valid");

	if (!params_loaded || !zone_okay)
	{
		ROS_WARN("Obstacle detection params not loaded");
		ros::shutdown();
	}

	else
	{
		ChangeTopic();
		timer = nh -> createTimer(ros::Duration(0.033), &ObstacleDetection::EmptyDataCallback, this);
	}

}

void ObstacleDetection::ChangeTopic()
{
	laser_sub_topic        = use_laser ? laser_sub_topic : "/empty_laser";
	ultrasonic_sub_topic   = use_ultrasonic ? ultrasonic_sub_topic : "/empty_ultra";
	front_camera_sub_topic = use_front_camera ? front_camera_sub_topic : "/empty_front_camera";
	back_camera_sub_topic  = use_back_camera ? back_camera_sub_topic : "/empty_back_camera";

	//Publishing empty data
	if(!use_laser)
	{
		source = laser;
		InitPointcloud(source);
		empty_laser = nh->advertise<sensor_msgs::LaserScan>("/empty_laser", 1);
	}

	if(!use_ultrasonic)
	{
		source = ultrasonic;
		InitPointcloud(source);
		empty_ultra = nh->advertise<sensor_msgs::LaserScan>("/empty_ultra", 1);
	}

	if(!use_front_camera)
	{
		source = front_camera;
		InitPointcloud(source);
		empty_front_camera = nh->advertise<sensor_msgs::PointCloud2>("/empty_front_camera", 1);
	}

	if(!use_back_camera)
	{
		source = back_camera;
		InitPointcloud(source);
		empty_back_camera = nh->advertise<sensor_msgs::PointCloud2>("/empty_back_camera", 1);
	}

	ROS_INFO_ONCE ("Subcribed Topics: %s, %s, %s, %s", laser_sub_topic.c_str(), ultrasonic_sub_topic.c_str(), front_camera_sub_topic.c_str(), back_camera_sub_topic.c_str());
}

void ObstacleDetection::InitSynchroniser()
{  
	vehicle_polygon_ = CalculateFootprints(vehicle);
	warning_zone_polygon_ = CalculateFootprints(warning[warning_zone_id]);
	stop_zone_polygon_ = CalculateFootprints(stop[stop_zone_id]);
	camera_polygon_ =  calculate_camera_box(camera);

	laserscan_sub = new message_filters::Subscriber<sensor_msgs::LaserScan>(*nh, laser_sub_topic, 1);
	ultrasonic_sub = new message_filters::Subscriber<sensor_msgs::LaserScan>(*nh, ultrasonic_sub_topic, 1);
	D415_front_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(*nh, front_camera_sub_topic, 1);
	D415_back_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(*nh, back_camera_sub_topic, 1);
	LaserUltraBothCameraSync = new message_filters::Synchronizer<ObstacleDetection::LaserUltraBothCameraPolicy>(ObstacleDetection::LaserUltraBothCameraPolicy(10),
			*laserscan_sub, *ultrasonic_sub, *D415_front_sub, *D415_back_sub);
	LaserUltraBothCameraSync->registerCallback(boost::bind(&ObstacleDetection::MessageSyncCallback, this, _1, _2, _3, _4));
}


void ObstacleDetection::EmptyDataCallback(const ros::TimerEvent& event)
{
	if(!use_laser)
	{
		e_laser.header.stamp = ros::Time::now();

		empty_laser.publish(e_laser);
	}

	if(!use_ultrasonic)
	{
		e_ultra.header.stamp = ros::Time::now();
		empty_ultra.publish(e_ultra);
	}

	if(!use_front_camera)
	{
		e_front_camera.header.stamp = ros::Time::now();
		empty_front_camera.publish(e_front_camera);
	}

	if(!use_back_camera)
	{
		e_back_camera.header.stamp = ros::Time::now();
		empty_back_camera.publish(e_back_camera);
	}
}

void ObstacleDetection::InitPointcloud(int i)
{
	switch(i)
	{

		case 0:
			e_laser.header.frame_id = empty_frame_id;

		case 1:
			e_ultra.header.frame_id = empty_frame_id;

		case 2:
			{
				e_front_camera.header.frame_id = empty_frame_id;

				sensor_msgs::PointCloud2Modifier modifier(e_front_camera);
				modifier.setPointCloud2FieldsByString(1, "xyz");

				sensor_msgs::PointCloud2Iterator<float>iter_x(e_front_camera, "x");
				sensor_msgs::PointCloud2Iterator<float>iter_y(e_front_camera, "y");
				sensor_msgs::PointCloud2Iterator<float>iter_z(e_front_camera, "z");
				break;
			}

		case 3:
			{
				e_back_camera.header.frame_id = empty_frame_id;

				sensor_msgs::PointCloud2Modifier modifier(e_back_camera);
				modifier.setPointCloud2FieldsByString(1, "xyz");

				sensor_msgs::PointCloud2Iterator<float>iter_x(e_back_camera, "x");
				sensor_msgs::PointCloud2Iterator<float>iter_y(e_back_camera, "y");
				sensor_msgs::PointCloud2Iterator<float>iter_z(e_back_camera, "z");
				break;
			}
	}
}

bool ObstacleDetection::ValidateSafetyZone(coordinates current, coordinates reference)
{
	if (current.minx > reference.minx && current.miny > reference.miny && current.minz > reference.minz)
	{
		if (current.maxx < reference.maxx && current.maxy < reference.maxy && current.maxz < reference.maxz)
		{
			return false;
		}
	}
	return true;
}

bool ObstacleDetection::SetObstacleZonesCallback(novus_msgs::SetObstacleZones::Request &req, novus_msgs::SetObstacleZones::Response &res)
{
	
	ROS_INFO("Zone selection service called for zone_id %d", req.zone_id);
	int zone_id = 0;
	std::string zone_name;

	zone_id = req.zone_id;
	zone_name = req.zone_name;

	if (zone_name == "WARNING" || zone_name == "warning")
	{
		std::string warning_zone_request_ = "/warning_zone" + std::to_string(zone_id);
		std::cout<< nh->hasParam(warning_zone_request_) <<std::endl;
		if(nh->hasParam(warning_zone_request_) && (zone_id < total_warning_zones))
		{
			warning_zone_id = zone_id;
			res.report = "SUCCESS";
			res.success = true;
			return true;
		}
		else
		{
			warning_zone_id = 0;
			res.success = false;
			res.report = "ERROR: " + warning_zone_request_ + " doesn't exist." + " Setting to Max Safety: Zone number 0.";
		}
      warning_zone_polygon_ = CalculateFootprints(warning[warning_zone_id]);
      camera.maxx = warning[warning_zone_id].maxx;
      camera.minx = warning[warning_zone_id].minx;
      camera_polygon_ =  calculate_camera_box(camera);

	}
	else if (zone_name == "STOP" || zone_name == "stop")
	{
		std::string stop_zone_request_ = "/stop_zone" + std::to_string(zone_id);
		if(nh->hasParam(stop_zone_request_) && (zone_id < total_stop_zones))
		{
			stop_zone_id = zone_id;
			res.report = "SUCCESS";
			res.success = true;
			return true;
		}
		else
		{
			stop_zone_id = 0;
			res.success = false;
			res.report = "ERROR: " + stop_zone_request_ + " doesn't exist." + " Setting to Max Safety: Zone number 0.";
		}

      stop_zone_polygon_ = CalculateFootprints(stop[stop_zone_id]);
	}
	else if (zone_name == "BOTH" || zone_name == "both")
	{
		std::string warning_zone_request_ = "/warning_zone" + std::to_string(zone_id);
		std::string stop_zone_request_ = "/stop_zone" + std::to_string(zone_id);
		if(nh->hasParam(warning_zone_request_) && nh->hasParam(stop_zone_request_) && zone_id < total_stop_zones && zone_id < total_warning_zones)
		{
			warning_zone_id = zone_id;
			stop_zone_id = zone_id;
			res.report = "SUCCESS";
			res.success = true;
			return true;
		}
		else
		{
			warning_zone_id = 0;
      stop_zone_id = 0;
			res.success = false;
			res.report = "ERROR: " + warning_zone_request_ + " AND/OR " + stop_zone_request_ + " do not exist." + "Setting both zones to Max Safety: Zone number 0.";
		}

      warning_zone_polygon_ = CalculateFootprints(warning[warning_zone_id]);
      stop_zone_polygon_ = CalculateFootprints(stop[stop_zone_id]);
      camera.maxx = warning[warning_zone_id].maxx;
      camera.minx = warning[warning_zone_id].minx;
      camera_polygon_ =  calculate_camera_box(camera);

	}
	else
	{
		ROS_ERROR("Allowed zone names are {'WARNING', 'STOP', 'BOTH'}");
		res.report = "ERROR: Allowed zone names are {'WARNING', 'STOP', 'BOTH'}";
	}
}

geometry_msgs::Polygon ObstacleDetection::CalculateFootprints(struct coordinates footprint)
{
	geometry_msgs::Point32 p[4];
	geometry_msgs::Polygon polygon_;

	p[0].x = footprint.maxx;
	p[0].y = footprint.maxy;

	p[1].x = footprint.maxx;
	p[1].y = footprint.miny;

	p[2].x = footprint.minx;
	p[2].y = footprint.miny;

	p[3].x = footprint.minx;
	p[3].y = footprint.maxy;

	for (int i = 0; i < 4; i++)
		p[i].z = 0.0;

	for (int i = 0; i < 4; i++)
		polygon_.points.push_back(p[i]);
	return polygon_;
}

//takes coordinates and calculates polygon (only for front camera)
geometry_msgs::Polygon ObstacleDetection::calculate_camera_box(struct coordinates footprint)
{
	geometry_msgs::Point32 p[4];
	geometry_msgs::Polygon polygon_;

	p[0].x = footprint.maxx;
	p[0].y = footprint.maxy;

	p[1].x = footprint.maxx;
	p[1].y = footprint.miny;

	p[2].x = footprint.minx;
	p[2].y = footprint.miny;

	p[3].x = footprint.minx;
	p[3].y = footprint.maxy;

	for (int i = 0; i < 4; i++)
		p[i].z = footprint.minz;;

	for (int i = 0; i < 4; i++)
		polygon_.points.push_back(p[i]);
	return polygon_;
}

void ObstacleDetection::MessageSyncCallback(const sensor_msgs::LaserScanConstPtr& laser_msg,
		const sensor_msgs::LaserScanConstPtr& ultrasonic_msg, const sensor_msgs::PointCloud2ConstPtr& D415_front_msg,
		const sensor_msgs::PointCloud2ConstPtr& D415_back_msg)
{

	if(!transforms_computed)
		ComputeTransforms(laser_msg, ultrasonic_msg, D415_front_msg, D415_back_msg);

	if (!transform_okay)
		return;

	// converting laser scan msg to pointcloud
	sensor_msgs::PointCloud2 laser_cloud, laser_cloud_transformed, ultrasonic_cloud, ultrasonic_cloud_transformed,
		D415_front_cloud_transformed, D415_back_cloud_transformed;

	pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ultrasonic_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr D415_front_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr D415_back_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr combined_camera_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr agg_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);

	projector.projectLaser(*laser_msg, laser_cloud);
	projector.projectLaser(*ultrasonic_msg, ultrasonic_cloud);

	pcl_ros::transformPointCloud(eigen_laser, laser_cloud, laser_cloud_transformed);
	pcl_ros::transformPointCloud(eigen_ultrasonic, ultrasonic_cloud, ultrasonic_cloud_transformed);
	pcl_ros::transformPointCloud(eigen_D415_front, *D415_front_msg, D415_front_cloud_transformed);
	pcl_ros::transformPointCloud(eigen_D415_back, *D415_back_msg, D415_back_cloud_transformed);

	laser_cloud_transformed.header.frame_id = vehicle_frame_id;
	ultrasonic_cloud_transformed.header.frame_id = vehicle_frame_id;
	D415_front_cloud_transformed.header.frame_id = vehicle_frame_id;
	D415_back_cloud_transformed.header.frame_id = vehicle_frame_id;


	pcl::fromROSMsg(laser_cloud_transformed, *laser_cloud_pcl);
	pcl::fromROSMsg(ultrasonic_cloud_transformed, *ultrasonic_cloud_pcl);
	pcl::fromROSMsg(D415_front_cloud_transformed, *D415_front_cloud_pcl);
	pcl::fromROSMsg(D415_back_cloud_transformed, *D415_back_cloud_pcl);

	combined_camera_cloud_pcl->header.frame_id = vehicle_frame_id;
	*combined_camera_cloud_pcl += *D415_front_cloud_pcl;
	*combined_camera_cloud_pcl += *D415_back_cloud_pcl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr camera_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropBox < pcl::PointXYZ > camera_filter;
	camera_filter.setMin(Eigen::Vector4f(camera.minx, camera.miny, camera.minz, 1.0));
	camera_filter.setMax(Eigen::Vector4f(camera.maxx,camera.maxy, camera.maxz, 1.0));
	camera_filter.setInputCloud(combined_camera_cloud_pcl);
	camera_filter.filter(*camera_filtered);

#if PUB_DEBUG
	pcl_pub_laser.publish(*ultrasonic_cloud_pcl);
	pcl_camera.publish(*camera_filtered);
#endif

	// Add all pointclouds

	*agg_cloud_pcl += *laser_cloud_pcl;
	*agg_cloud_pcl += *ultrasonic_cloud_pcl;
	*agg_cloud_pcl += *camera_filtered;

	agg_cloud_pcl->header.frame_id = vehicle_frame_id;

#if PUB_DEBUG
	pcl_pub_agg.publish(*agg_cloud_pcl);
#endif

	detect_obstacles(agg_cloud_pcl);

}

void ObstacleDetection::detect_obstacles(pcl::PointCloud<pcl::PointXYZ>::Ptr agg_cloud_pcl)
{
	//Creating pointclouds for visualization
	pcl::PointCloud<pcl::PointXYZ>::Ptr warning_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr stop_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);

	// Limiting data to warning zone range
	pcl::CropBox < pcl::PointXYZ > bound_filter;
	bound_filter.setMin(
			Eigen::Vector4f(warning[warning_zone_id].minx, warning[warning_zone_id].miny, warning[warning_zone_id].minz, 1.0));
	bound_filter.setMax(
			Eigen::Vector4f(warning[warning_zone_id].maxx, warning[warning_zone_id].maxy, warning[warning_zone_id].maxz, 1.0));
	bound_filter.setInputCloud(agg_cloud_pcl);
	bound_filter.filter(*warning_cloud_pcl);

#if PUB_DEBUG
	ROS_INFO("warning cloud size: %d", warning_cloud_pcl->points.size());
	pcl_pub_warning.publish(*warning_cloud_pcl);
#endif

	// Clearing Vehicle Cube
	pcl::CropBox < pcl::PointXYZ > vehicle_filter;
	vehicle_filter.setMin(Eigen::Vector4f(vehicle.minx, vehicle.miny, vehicle.minz, 1.0));
	vehicle_filter.setMax(Eigen::Vector4f(vehicle.maxx, vehicle.maxy, vehicle.maxz, 1.0));
	vehicle_filter.setNegative(true);
	vehicle_filter.setInputCloud(warning_cloud_pcl);
	vehicle_filter.filter(*warning_cloud_pcl);


#if PUB_DEBUG
	pcl_pub_wo_veh.publish(*warning_cloud_pcl);
#endif
	//Stop zone
	pcl::CropBox < pcl::PointXYZ > stop_filter;
	stop_filter.setMin(Eigen::Vector4f(stop[stop_zone_id].minx, stop[stop_zone_id].miny, stop[stop_zone_id].minz, 1.0));
	stop_filter.setMax(Eigen::Vector4f(stop[stop_zone_id].maxx, stop[stop_zone_id].maxy, stop[stop_zone_id].maxz, 1.0));
	stop_filter.setInputCloud(warning_cloud_pcl);
	stop_filter.filter(*stop_cloud_pcl);
	stop_cloud_pcl->header.frame_id = vehicle_frame_id;

#if PUB_DEBUG
	pcl_pub_stop.publish(*stop_cloud_pcl);
#endif

	//Deciding on the current obstacle zone
	int obstacle_zone = 0;
	if (stop_cloud_pcl->points.size() > 3)
		obstacle_zone = 1;
	else if (warning_cloud_pcl->points.size() > 3)
		obstacle_zone = 2;

	//Implementing obstacle persistence. Obstacle is not cleared until persistence timeout.

	if (obstacle_zone == 1)
		last_obstacle_trigger = ros::Time::now();
	else if(obstacle_zone == 2)
	{
		ros::Time curr_time = ros::Time::now();
		if ((curr_time.toSec() - last_obstacle_trigger.toSec()) < obstacle_persistence)
			obstacle_zone = 1;
		else
			last_warning_trigger = ros::Time::now();
	}
	else
	{
		ros::Time curr_time = ros::Time::now();
		if ((curr_time.toSec() - last_obstacle_trigger.toSec()) < obstacle_persistence)
			obstacle_zone = 1;
		else if((curr_time.toSec() - last_warning_trigger.toSec()) < obstacle_persistence)
			obstacle_zone = 2;
	}

	std_msgs::Int16 obstacle_msg;
	obstacle_msg.data = obstacle_zone;

	obstacle_zone_pub.publish(obstacle_msg);

	std_msgs::Int8 current_safety_zone_msg;

	//publish current warning zone
	current_safety_zone_msg.data = warning_zone_id;
	current_warning_zone_pub.publish(current_safety_zone_msg);

	//publish current stop zone
	current_safety_zone_msg.data = stop_zone_id;
	current_stop_zone_pub.publish(current_safety_zone_msg);


	geometry_msgs::PolygonStamped footprint_;

	footprint_ = StampedFootprints(vehicle_polygon_);
	vehicle_polygon_pub.publish(footprint_);

	footprint_ = StampedFootprints(warning_zone_polygon_);
	warning_zone_polygon_pub.publish(footprint_);

	footprint_ = StampedFootprints(stop_zone_polygon_);
	stop_zone_polygon_pub.publish(footprint_);

	footprint_ = StampedFootprints(camera_polygon_);
	camera_polygon_pub.publish(footprint_);

	end_time = ros::Time::now().toSec();


}

geometry_msgs::PolygonStamped ObstacleDetection::StampedFootprints(geometry_msgs::Polygon polygon_)
{
	geometry_msgs::PolygonStamped polygon_stamped;
	polygon_stamped.header.stamp = ros::Time::now();
	polygon_stamped.header.frame_id = vehicle_frame_id;
	polygon_stamped.polygon = polygon_;

	return polygon_stamped;
}

void ObstacleDetection::ComputeTransforms(const sensor_msgs::LaserScanConstPtr& laser_msg,
		const sensor_msgs::LaserScanConstPtr& ultrasonic_msg, const sensor_msgs::PointCloud2ConstPtr& D415_front_msg,
		const sensor_msgs::PointCloud2ConstPtr& D415_back_msg)
{
	bool exception_occured = false;

	try
	{
		ros::Time lookup_time = ros::Time(0);
		listener->waitForTransform(laser_msg->header.frame_id, vehicle_frame_id, lookup_time, ros::Duration(0.1));
		listener->waitForTransform(ultrasonic_msg->header.frame_id, vehicle_frame_id, lookup_time, ros::Duration(0.1));
		listener->waitForTransform(D415_front_msg->header.frame_id, vehicle_frame_id, lookup_time, ros::Duration(0.1));
		listener->waitForTransform(D415_back_msg->header.frame_id, vehicle_frame_id, lookup_time, ros::Duration(0.1));
		listener->lookupTransform(vehicle_frame_id, laser_msg->header.frame_id, lookup_time, transform_laser);
		listener->lookupTransform(vehicle_frame_id, ultrasonic_msg->header.frame_id, lookup_time, transform_ultrasonic);
		listener->lookupTransform(vehicle_frame_id, D415_front_msg->header.frame_id, lookup_time, transform_D415_front);
		listener->lookupTransform(vehicle_frame_id, D415_back_msg->header.frame_id, lookup_time, transform_D415_back);

		pcl_ros::transformAsMatrix(transform_laser, eigen_laser);
		pcl_ros::transformAsMatrix(transform_ultrasonic, eigen_ultrasonic);
		pcl_ros::transformAsMatrix(transform_D415_front, eigen_D415_front);
		pcl_ros::transformAsMatrix(transform_D415_back, eigen_D415_back);
	} 

	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		ros::Duration(0.1).sleep();
		exception_occured = true;
	}

	if (exception_occured == false)
	{
		transforms_computed = true;
	}

	if (!transforms_computed)
	{
		transform_okay = false;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "obstacle_detection_node");
	ros::MultiThreadedSpinner spinner(2);

	ObstacleDetection obstacle_detection;
	obstacle_detection.load_params();

	obstacle_detection.InitSynchroniser();

	while (ros::ok())
	{
		//spin as fast as possible
		spinner.spin();
	}
	return 0;
}
