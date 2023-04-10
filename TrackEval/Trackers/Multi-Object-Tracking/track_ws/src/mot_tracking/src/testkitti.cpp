#include <iostream>
#include <vector>
#include <stdlib.h>
#include <string>
#include <chrono>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>
#include <time.h>
#include <unordered_map>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Boost
#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
// #include "lonlat2utm.h"

#include "readparam.h"
#include "tracker.h"

// Ros
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#include "data_source/kitti_reader.h"

using namespace std;

static string folder_lidar = "results/";
static string format_lidar = "%|06|.txt";

static int64_t gtm()
{
	struct timeval tm;
	gettimeofday(&tm, 0);
	// return ms
	int64_t re = (((int64_t)tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
	return re;
}

struct pose
{
	double x;
	double y;
	double heading;
};

template <typename T>
string toString(const T &t)
{
	ostringstream oss;
	oss << t;
	return oss.str();
}

cv::Point cloud2camera(Eigen::Vector3d input)
{
	Eigen::Matrix4d RT_velo_to_cam;
	Eigen::Matrix4d R_rect;
	Eigen::MatrixXd project_matrix(3, 4);
	RT_velo_to_cam << 7.49916597e-03, -9.99971248e-01, -8.65110297e-04, -6.71807577e-03,
		1.18652889e-02, 9.54520517e-04, -9.99910318e-01, -7.33152811e-02,
		9.99882833e-01, 7.49141178e-03, 1.18719929e-02, -2.78557062e-01,
		0, 0, 0, 1;
	R_rect << 0.99992475, 0.00975976, -0.00734152, 0,
		-0.0097913, 0.99994262, -0.00430371, 0,
		0.00729911, 0.0043753, 0.99996319, 0,
		0, 0, 0, 1;
	project_matrix << 7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01,
		0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01,
		0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03;
	Eigen::MatrixXd transform_matrix_ = project_matrix * R_rect * RT_velo_to_cam;

	Eigen::Vector4d point;
	point << input(0), input(1), input(2), 1;
	Eigen::Vector3d pimage = transform_matrix_ * point;
	cv::Point p2d = cv::Point(pimage(0) / pimage(2), pimage(1) / pimage(2));
	return p2d;
}

void draw3dbox(Detect &det, cv::Mat &image, vector<int> &color)
{
	float h = det.box[0];
	float w = det.box[1];
	float l = det.box[2];
	float x = det.position[0];
	float y = det.position[1];
	float z = det.z;
	float yaw = -det.yaw - 90 * M_PI / 180;
	double boxroation[9] = {cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1};

	Eigen::MatrixXd BoxRotation = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(boxroation);
	double xAxisP[8] = {l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2};
	double yAxisP[8] = {w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2};
	double zAxisP[8] = {h, h, h, h, 0, 0, 0, 0};
	vector<cv::Point> imagepoint;
	Eigen::Vector3d translation(x, y, z);

	for (int i = 0; i < 8; i++)
	{
		Eigen::Vector3d point_3d(xAxisP[i], yAxisP[i], zAxisP[i]);
		Eigen::Vector3d rotationPoint_3d = BoxRotation * point_3d + translation;
		cv::Point imgpoint = cloud2camera(rotationPoint_3d);
		imagepoint.push_back(imgpoint);
	}

	int r = color[0];
	int g = color[1];
	int b = color[2];

	cv::line(image, imagepoint[0], imagepoint[1], cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[1], imagepoint[2], cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[2], imagepoint[3], cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[3], imagepoint[0], cv::Scalar(r, g, b), 2, CV_AA);

	cv::line(image, imagepoint[4], imagepoint[5], cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[5], imagepoint[6], cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[6], imagepoint[7], cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[7], imagepoint[4], cv::Scalar(r, g, b), 2, CV_AA);

	cv::line(image, imagepoint[0], imagepoint[4], cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[1], imagepoint[5], cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[2], imagepoint[6], cv::Scalar(r, g, b), 2, CV_AA);
	cv::line(image, imagepoint[3], imagepoint[7], cv::Scalar(r, g, b), 2, CV_AA);

	cv::line(image, imagepoint[0], imagepoint[5], cv::Scalar(226, 43, 138), 2, CV_AA);
	cv::line(image, imagepoint[1], imagepoint[4], cv::Scalar(226, 43, 138), 2, CV_AA);

	imagepoint.clear();
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pubimage = it.advertise("/mot_tracking/image", 1);
	ros::Publisher publidar = nh.advertise<sensor_msgs::PointCloud2>("/mot_tracking/pointcloud", 1);
	ros::Publisher pubmarker = nh.advertise<visualization_msgs::MarkerArray>("/mot_tracking/box", 1);
	ros::Publisher pubtextmarker = nh.advertise<visualization_msgs::MarkerArray>("/mot_tracking/id", 1);

	// tracking class
	string trackclass;
	ros::param::get("/tracking_node/trackclass", trackclass);

	ros::Rate r(10);

	Param param;
	Tracker tracker(param);

	//==============setup data source==================
	// data path
	string datapath;
	ros::param::get("/tracking_node/datapath", datapath);

	// data number
	string file;
	ros::param::get("/tracking_node/file", file);

	std::string path = datapath + file;
	std::cout << path << endl;

	Kitti_Reader kitti(path, file);

	boost::char_separator<char> sep{" "};
	tokenizer tokn(trackclass, sep);
    vector<string> temp_sep(tokn.begin(), tokn.end());
	for (string s : temp_sep)
		kitti.setTrackClass(s);

	kitti.loadAll();

	double maxframe = kitti.getMaxFrame();
	std::cout << "maxframe " << maxframe << endl;

	// read the label file

	// start position
	Eigen::Matrix3d rotZorigion;
	Eigen::Isometry3d porigion;
	double oriheading = 0;
	double orix = 0;
	double oriy = 0;

	Eigen::Matrix3d rotZpre;
	Eigen::Isometry3d ppre;
	double preheading = 0;
	double prex = 0;
	double prey = 0;
	cv::Mat images = cv::Mat::zeros(608, 608, CV_8UC3);

	double totaltime = 0;
	float time = 0;
	int frame = 0;

	cv::RNG rng(12345);
	unordered_map<int, vector<int>> idcolor;

	/*cv::VideoWriter writer;

	writer.open(path+"tracking.avi",CV_FOURCC('M', 'J', 'P', 'G'), 10 , cv::Size(1241, 376), true);
		if (!writer.isOpened())
		{
			std::abort();
		}*/

	while (ros::ok() && frame < maxframe)
	{
		cout << "||||||||||||Frame" << frame << "|||||||||||||" << endl;
	/*
		std::cout << "Press 'n' to continue...\n";
		std::string input;
		while (true)
		{
			std::cin >> input;
			if (input == "n")
			{
				break;
			}
		}
	*/
		int max_marker_size_ = 50;
		visualization_msgs::MarkerArray marker_array;
		marker_array.markers.clear();
		visualization_msgs::Marker bbox_marker;
		bbox_marker.header.frame_id = "global_init_frame";
		bbox_marker.header.stamp = ros::Time::now();
		bbox_marker.ns = "";
		bbox_marker.lifetime = ros::Duration();
		bbox_marker.frame_locked = true;
		bbox_marker.type = visualization_msgs::Marker::CUBE;
		bbox_marker.action = visualization_msgs::Marker::ADD;

		visualization_msgs::MarkerArray text_marker_array;
		text_marker_array.markers.clear();
		visualization_msgs::Marker text_marker;
		text_marker.header.frame_id = "global_init_frame";
		text_marker.header.stamp = ros::Time::now();
		text_marker.ns = "";
		text_marker.lifetime = ros::Duration();
		text_marker.frame_locked = true;
		text_marker.color.r = 0.0f;
		text_marker.color.g = 0.0f;
		text_marker.color.b = 0.0f;
		text_marker.color.a = 0.9;
		text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		text_marker.action = visualization_msgs::Marker::ADD;

		// get the gps data and get the tranformation matrix
		cout << "read gps" << endl;
		double twosub = 0;
		Eigen::Isometry3d translate2origion;
		Eigen::Isometry3d origion2translate;
		double UTME, UTMN;
		double latitude, longitude, heading;
		kitti.getGPS(latitude, longitude, heading, UTME, UTMN, frame);
		cout << latitude << longitude << heading << UTME << UTMN << endl;

		// read image
		cout << "read image" << endl;
		cv::Mat rgbimage;
		kitti.getRBGImage(rgbimage, frame);

		// read cloud
		cout << "read cloud" << endl;
		pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud(
			new pcl::PointCloud<pcl::PointXYZI>);
		kitti.getPointcloud(*Cloud, frame);

		// read detections
		cout << "read detections" << endl;
		vector<Detect> inputDets;
		kitti.getLabel(inputDets, frame);

		if (frame == 0)
		{
			cout << "Frame0" << endl;
			Eigen::AngleAxisd roto(heading, Eigen::Vector3d::UnitZ());
			rotZorigion = roto.toRotationMatrix();
			porigion = rotZorigion;
			orix = UTME;
			oriy = UTMN;
			porigion.translation() = Eigen::Vector3d(UTME, UTMN, 0);
			oriheading = heading;

			preheading = heading;
			prex = UTME;
			prey = UTMN;
			rotZpre = rotZorigion;
			ppre = rotZpre;
			ppre.translation() = Eigen::Vector3d(UTME, UTMN, 0);
		}
		else
		{

			Eigen::AngleAxisd rotnow(heading, Eigen::Vector3d::UnitZ());
			Eigen::Matrix3d rotpnow = rotnow.toRotationMatrix();
			Eigen::Isometry3d p2;
			p2 = rotpnow;
			p2.translation() = Eigen::Vector3d(UTME, UTMN, 0);
			translate2origion = porigion.inverse() * p2;
			origion2translate = p2.inverse() * porigion;
			twosub = heading - oriheading;
		}

		int size = inputDets.size();
		std::cout << "inputsize " << size << endl;

		for (int i = 0; i < size; ++i)
		{

			vector<int> color = {0, 255, 0};
			draw3dbox(inputDets[i], rgbimage, color);

			std::cout << "input: " << inputDets[i].position(0) << " " << inputDets[i].position(1) << " " << inputDets[i].z << " " << inputDets[i].box[0] << " " << inputDets[i].box[1] << " " << inputDets[i].box[2] << endl;

			Eigen::VectorXd v(2, 1);
			v(1) = inputDets[i].position(0);  // x in kitti lidar
			v(0) = -inputDets[i].position(1); // y in kitti lidar
			if (frame != 0)
			{
				Eigen::Vector3d p_0(v(0), v(1), 0);
				Eigen::Vector3d p_1;
				p_1 = translate2origion * p_0;
				v(0) = p_1(0);
				v(1) = p_1(1);
			}

			inputDets[i].position(0) = v(0);
			inputDets[i].position(1) = v(1);

			inputDets[i].rotbox = cv::RotatedRect(cv::Point2f((v(0) + 25) * 608 / 50, v(1) * 608 / 50),
												  cv::Size2f(inputDets[i].box[1] * 608 / 50, inputDets[i].box[2] * 608 / 50), inputDets[i].yaw);

			cv::RotatedRect detshow = cv::RotatedRect(cv::Point2f((v(0) + 25) * 608 / 50, v(1) * 608 / 50),
													  cv::Size2f(inputDets[i].box[1] * 608 / 50, inputDets[i].box[2] * 608 / 50), inputDets[i].yaw);
			cv::Point2f vertices[4];
			detshow.points(vertices);
			for (int j = 0; j < 4; j++)
				cv::line(images, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 0, 255), 1);
		}

		std::vector<Eigen::VectorXd> result;
		int64_t tm0 = gtm();
		std::cout << "start tracking" << std::endl;
		tracker.track(inputDets, time, result);
		int64_t tm1 = gtm();
		printf("[INFO]update cast time:%ld us\n", tm1 - tm0);

		double x = tm1 - tm0;
		totaltime += x;

		int marker_id = 0;
		for (int i = 0; i < result.size(); ++i)
		{
			Eigen::VectorXd r = result[i];
			if (frame != 0)
			{
				Eigen::Vector3d p_0(r(1), r(2), 0);
				Eigen::Vector3d p_1;
				p_1 = origion2translate * p_0;
				r(1) = p_1(0);
				r(2) = p_1(1);
			}

			Detect det;
			det.box2D.resize(4);
			det.box.resize(3);
			det.box[0] = r(9); // h
			det.box[1] = r(8); // w
			det.box[2] = r(7); // l
			det.z = r(10);
			det.yaw = r(6);
			det.position = Eigen::VectorXd(2);
			det.position << r(2), -r(1);
			std::cout << "det: " << det.position(0) << " " << det.position(1) << " " << det.z << " " << det.box[0] << " " << det.box[1] << " " << det.box[2] << endl;

			if (!idcolor.count(int(r(0))))
			{
				int red = rng.uniform(0, 255);
				int green = rng.uniform(0, 255);
				int blue = rng.uniform(0, 255);
				idcolor[int(r(0))] = {red, green, blue};
			}
			draw3dbox(det, rgbimage, idcolor[int(r(0))]);

			Eigen::Vector3f eulerAngle(-det.yaw, 0.0, 0.0);
			Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(2), Eigen::Vector3f::UnitX()));
			Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1), Eigen::Vector3f::UnitY()));
			Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(0), Eigen::Vector3f::UnitZ()));
			const Eigen::Quaternionf bboxQ1(yawAngle * pitchAngle * rollAngle);
			Eigen::Vector4f q = bboxQ1.coeffs();

			bbox_marker.id = marker_id;
			bbox_marker.pose.position.x = det.position(0);
			bbox_marker.pose.position.y = det.position(1);
			bbox_marker.pose.position.z = det.z + det.box[0] / 2;
			bbox_marker.pose.orientation.x = q[0];
			bbox_marker.pose.orientation.y = q[1];
			bbox_marker.pose.orientation.z = q[2];
			bbox_marker.pose.orientation.w = q[3];
			bbox_marker.scale.x = det.box[1];
			bbox_marker.scale.y = det.box[2];
			bbox_marker.scale.z = det.box[0];
			bbox_marker.color.r = float(idcolor[int(r(0))][0]) / 255;
			bbox_marker.color.g = float(idcolor[int(r(0))][1]) / 255;
			bbox_marker.color.b = float(idcolor[int(r(0))][2]) / 255;
			bbox_marker.color.a = 0.8;
			marker_array.markers.push_back(bbox_marker);

			text_marker.id = marker_id;
			text_marker.pose.position.x = det.position(0);
			text_marker.pose.position.y = det.position(1);
			text_marker.pose.position.z = det.z + det.box[0] / 2 + 1;
			text_marker.pose.orientation.x = 0;
			text_marker.pose.orientation.y = 0;
			text_marker.pose.orientation.z = 0;
			text_marker.pose.orientation.w = 1;
			text_marker.scale.x = 0.2;
			text_marker.scale.y = 0;
			text_marker.scale.z = 1;
			text_marker.text = "ID: " + toString(int(r(0)));
			text_marker_array.markers.push_back(text_marker);
			++marker_id;
		}

		if (marker_array.markers.size() > max_marker_size_)
		{
			max_marker_size_ = marker_array.markers.size();
		}

		for (size_t i = marker_id; i < max_marker_size_; ++i)
		{
			bbox_marker.id = i;
			bbox_marker.color.a = 0;
			bbox_marker.pose.position.x = 0;
			bbox_marker.pose.position.y = 0;
			bbox_marker.pose.position.z = 0;
			bbox_marker.scale.x = 0;
			bbox_marker.scale.y = 0;
			bbox_marker.scale.z = 0;
			marker_array.markers.push_back(bbox_marker);

			text_marker.id = i;
			text_marker.color.a = 0;
			text_marker.pose.position.x = 0;
			text_marker.pose.position.y = 0;
			text_marker.pose.position.z = 0;
			text_marker.scale.x = 0;
			text_marker.scale.y = 0;
			text_marker.scale.z = 0;
			text_marker_array.markers.push_back(text_marker);

			++marker_id;
		}

		cv::imshow("x", rgbimage);
		cv::waitKey(1);
		// writer<<rgbimage;
		sensor_msgs::ImagePtr imgmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbimage).toImageMsg(); // 转换为ros接受的消息
		pubimage.publish(imgmsg);

		sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*Cloud, ros_cloud);
		ros_cloud.header.frame_id = "global_init_frame";
		publidar.publish(ros_cloud);

		pubtextmarker.publish(text_marker_array);
		pubmarker.publish(marker_array);

		time += 0.1;
		frame++;
		r.sleep();
	}
	// writer.release();

	return 0;
}
