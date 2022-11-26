#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <tf/transform_listener.h>
//PCL related includes
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
//Includes for laser scan projection into pointcloud
#include <laser_geometry/laser_geometry.h>
//Includes for service server
#include <novus_msgs/SetObstacleZones.h>
#include <geometry_msgs/PolygonStamped.h>

#include<fstream>
#include <iostream>

// #include <dynamic_reconfigure/client.h>

// // Global variable for the example
// using realsense_dyn_reconfigure = dynamic_reconfigure::Client<realsense2_camera::rs435_paramsConfig>;
// boost::shared_ptr<realsense_dyn_reconfigure> realsense_params;

using namespace std;

class ObstacleDetection
{

private:
		
	struct coordinates
	{
	  double minx;
	  double miny;
	  double minz;
	  double maxx;
	  double maxy;
	  double maxz;
	};

	enum ObservationSources
	{
		laser,
		ultrasonic,
		front_camera,
		back_camera
	};

	//ideally size of array should be at least total_number_of_zones or greater than that
	coordinates vehicle, camera ;
	std::vector<coordinates> warning, stop;
	ObservationSources source; 

	//ros-related variables
	ros::NodeHandle* nh;
	ros::Publisher obstacle_zone_pub, current_warning_zone_pub, current_stop_zone_pub;
    
    ros::Publisher empty_laser, empty_ultra, empty_front_camera, empty_back_camera;

	ros::Subscriber laser_sub, front_camera_sub, back_camera_sub, ultra_sub;

	ros::Time last_obstacle_trigger, last_warning_trigger;

	ros::ServiceServer zone_select_server_instance;

	#if PUB_DEBUG
	ros::Publisher pcl_pub_wo_veh;
	ros::Publisher pcl_pub_warning;
	ros::Publisher pcl_pub_stop;
	ros::Publisher pcl_pub_agg;
	ros::Publisher pcl_pub_laser;
	ros::Publisher pcl_camera;
	#endif

	ros::Publisher vehicle_polygon_pub, warning_zone_polygon_pub, stop_zone_polygon_pub, camera_polygon_pub;

	//tf-related variables
	tf::TransformListener *listener;
	tf::StampedTransform transform_laser, transform_ultrasonic, transform_D415_front, transform_D415_back;
	Eigen::Matrix4f eigen_laser, eigen_ultrasonic, eigen_D415_front, eigen_D415_back;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan,
	    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> LaserUltraBothCameraPolicy;

	message_filters::Subscriber<sensor_msgs::LaserScan> *laserscan_sub;
	message_filters::Subscriber<sensor_msgs::LaserScan> *ultrasonic_sub;
	message_filters::Subscriber<sensor_msgs::PointCloud2> *D415_front_sub;
	message_filters::Subscriber<sensor_msgs::PointCloud2> *D415_back_sub;

	
	message_filters::Synchronizer<LaserUltraBothCameraPolicy> *LaserUltraBothCameraSync;

	//variables from paramaeter server
	double minimum_obstacle_size;
	int total_warning_zones, total_stop_zones;
	double obstacle_persistence;
	bool use_laser, use_front_camera, use_back_camera, use_ultrasonic;
	//default topic names
	std::string laser_sub_topic,ultrasonic_sub_topic, front_camera_sub_topic ,back_camera_sub_topic;

	//other variables
	int warning_zone_id, stop_zone_id;
	bool transforms_computed, transform_okay;
	std::string vehicle_frame_id, empty_frame_id;
	laser_geometry::LaserProjection projector;
	geometry_msgs::Polygon vehicle_polygon_, warning_zone_polygon_, stop_zone_polygon_, camera_polygon_ ;

	sensor_msgs::PointCloud2 e_front_camera, e_back_camera;
	sensor_msgs::LaserScan e_laser, e_ultra;


	ros::Timer timer;

	double start_time, end_time;

    fstream cycle_time;

public:

	ObstacleDetection();
	
	void InitSynchroniser();

	void load_params();

	bool ValidateSafetyZone(coordinates current, coordinates reference);

	bool SetObstacleZonesCallback(novus_msgs::SetObstacleZones::Request &req, novus_msgs::SetObstacleZones::Response &res);

	geometry_msgs::Polygon CalculateFootprints(struct coordinates footprint);
    
    geometry_msgs::Polygon calculate_camera_box(struct coordinates footprint);

    geometry_msgs::PolygonStamped StampedFootprints(geometry_msgs::Polygon polygon_);

    void MessageSyncCallback(const sensor_msgs::LaserScanConstPtr& laser_msg, const sensor_msgs::LaserScanConstPtr& ultrasonic_msg, const sensor_msgs::PointCloud2ConstPtr& D415_front_msg, const sensor_msgs::PointCloud2ConstPtr& D415_back_msg);

    void EmptyDataCallback(const ros::TimerEvent& event);

	void ChangeTopic();    

	void ComputeTransforms(const sensor_msgs::LaserScanConstPtr& laser_msg, const sensor_msgs::LaserScanConstPtr& ultrasonic_msg, const sensor_msgs::PointCloud2ConstPtr& D415_front_msg, const sensor_msgs::PointCloud2ConstPtr& D415_back_msg);

    void detect_obstacles(pcl::PointCloud<pcl::PointXYZ>::Ptr agg_cloud_pcl);

    void InitPointcloud(int i);
};
