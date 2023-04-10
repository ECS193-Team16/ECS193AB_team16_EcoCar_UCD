
#ifndef KITTI_READER_H
#define KITTI_READER_H

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
#include "lonlat2utm.h"

// #include "readparam.h"
#include "../tracker.h"

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
#include <opencv2/opencv.hpp>

#include "file_utils.h"

#define max_truncation 0
#define max_occlusion 2

typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
using namespace std;
class Kitti_Reader
{
public:
    Kitti_Reader(string &path, string &index) : _path(path), _index(index)
    {
            classname2id["DontCare"] = 0;
            classname2id["Car"] = 1;
            classname2id["Pedestrian"] = 2;
            classname2id["Cyclist"] = 3;
            memset(_trackclass, 0, sizeof _trackclass);;
        //cout<<"trackclass"<<_trackclass[0]<<endl;
        float time = 0.1;
        _imageFolder = _path + "/image_02/" + _index + "/";
        _lidarFolder = _path + "/velodyne/" + _index + "/";
        _gpsFile = path + "/oxts/" + _index + ".txt";
        _labelFile = path + "/label_02/" + _index + ".txt";

        // path checking
        boost::filesystem::path result_dir_path = boost::filesystem::path(_path);
        if (!boost::filesystem::exists(result_dir_path))
        {
            boost::filesystem::create_directories(result_dir_path);
        }

        
    }

    bool loadAll(){
        // load folders
       cout << "load image" << endl;
        _frame_count = Load_Camera_Data_Path(imageName, _imageFolder);
        if (!_frame_count)
        {
            std::cout << "image file wrong!" << endl;
            std::abort();
        }
        cout << "load lidar" << endl;
        if (_frame_count != Load_Lidar_Data_Path(lidarName, _lidarFolder))
        {
            std::cout << "lidar file wrong!" << endl;
            std::abort();
        }
        cout << "load gps" << endl;
        if (_frame_count != load_GPS_Data(_gpsFile))
        {
            std::cout << "gps file wrong!" << endl;
            std::abort();
        }
        cout << "load label" << endl;
        if (_frame_count != load_Label_Data(_labelFile))
        {
            std::cout << "Detecion file wrong!" << endl;
            std::cout << "(But leave it for now)" << endl;
            // std::abort();
        }

        _frame_count = lidarName.size();
        return true;
    }
    double getMaxFrame()
    {
        return _frame_count;
    }

    bool getRBGImage(cv::Mat &target, double frame)
    {
        string impath = _imageFolder + imageName[frame];
        cout << "read " << impath << endl;
        cv::Mat img = cv::imread(impath);
        if (img.empty())
        {
            // Failed to read image
            return false;
        }
        img.copyTo(target);
        return true;
    }

    bool getPointcloud(pcl::PointCloud<pcl::PointXYZI> &target, double frame)
    {
        string lidar_filename_path = _lidarFolder + lidarName[frame];
        cout << "read " << lidar_filename_path << endl;
        ifstream inputfile;
        inputfile.open(lidar_filename_path, ios::binary);
        if (!inputfile)
        {
            cerr << "ERROR: Cannot open file " << lidar_filename_path
                 << "! Aborting..." << endl;
            return false;
        }

        inputfile.seekg(0, ios::beg);
        for (int i = 0; inputfile.good() && !inputfile.eof(); i++)
        {
            pcl::PointXYZI data;
            inputfile.read(reinterpret_cast<char *>(&data), sizeof(data));
            pcl::PointXYZI p;
            p.x = data.x;
            p.y = data.y;
            p.z = data.z;
            p.intensity = data.intensity;
            // std::cout<<p.x<<" "<<p.y<<" "<<p.z<<" "<<endl;
            target.points.push_back(p);
        }
        return true;
    }

    bool getGPS(double &latitude, double &longitude, double &heading, double &UTME, double &UTMN, double frame)
    {

        boost::char_separator<char> sep{" "};
        tokenizer tokn(gpsdata[frame], sep);
        vector<string> temp_sep(tokn.begin(), tokn.end());

        latitude = stringToNum<double>(temp_sep[0]);
        longitude = stringToNum<double>(temp_sep[1]);
        heading = stringToNum<double>(temp_sep[5]) - 90 * M_PI / 180;
        LonLat2UTM(longitude, latitude, UTME, UTMN);
        cout << longitude << latitude << UTME << UTMN << endl;
        return true;
    }

    bool getLabel(std::vector<Detect> &targetdetects, double frame)
    {
        targetdetects = labeldata[frame];
        return true;
    }

    void setTrackClass(string trackclass)
    {
        _trackclass[classname2id[trackclass]]=true;
        cout<<"typei"<<trackclass<<classname2id[trackclass]<<_trackclass[classname2id[trackclass]]<<endl;
    }
    ~Kitti_Reader() = default;

private:
    string _path, _index;
    string _imageFolder;
    string _lidarFolder;
    string _gpsFile;
    string _labelFile;

    double _frame_count;

    std::vector<std::string> lidarName;
    std::vector<std::string> imageName;
    std::vector<std::string> gpsdata;
    unordered_map<int, vector<Detect>> labeldata;

    bool _trackclass[4];
    std::unordered_map<std::string, int> classname2id;

    double Load_Camera_Data_Path(std::vector<std::string> &imagefile_name, string image_folder_path)
    {
        std::cout << image_folder_path << endl;
        if (!get_all_files(image_folder_path, imagefile_name))
            return 0;
        return imagefile_name.size();
    }

    double Load_Lidar_Data_Path(std::vector<std::string> &lidarfile_name, string lidar_folder_path)
    {
        std::cout << lidar_folder_path << endl;
        if (!get_all_files(lidar_folder_path, lidarfile_name))
            return 0;
        return lidarfile_name.size();
    }

    double load_GPS_Data(string gps_file_path)
    {
        cout << gps_file_path << endl;
        std::ifstream gps(gps_file_path);
        if (gps)
        {
            boost::char_separator<char> sep_line{"\n"};
            std::stringstream buffer;
            buffer << gps.rdbuf();
            std::string contents(buffer.str());
            tokenizer tok_line(contents, sep_line);
            std::vector<std::string> lines(tok_line.begin(), tok_line.end());
            gpsdata = lines;
            return gpsdata.size();
        }
        return 0;
    }
    double load_Label_Data(string label_file_path)
    {
        cout << label_file_path << endl;
        std::ifstream label(label_file_path);
        if (label)
        {
            boost::char_separator<char> sep_line{"\n"};
            std::stringstream buffer;
            buffer << label.rdbuf();
            std::string contents(buffer.str());
            tokenizer tok_line(contents, sep_line);
            std::vector<std::string> lines(tok_line.begin(), tok_line.end());
            boost::char_separator<char> sep{" "};

            

            for (int i = 0; i < lines.size(); ++i)
            {
                tokenizer tokn(lines[i], sep);
                vector<string> temp_sep(tokn.begin(), tokn.end());
                int fra = stringToNum<int>(temp_sep[0]);        // frame
                string type = temp_sep[2];                      // class
                int truncation = stringToNum<int>(temp_sep[3]); // truncation
                int occlusion = stringToNum<int>(temp_sep[4]);  // occlusion
                float x1 = stringToNum<float>(temp_sep[6]);     // left[pixel]
                float y1 = stringToNum<float>(temp_sep[7]);     // top[pixel]
                float x2 = stringToNum<float>(temp_sep[8]);     // right[pixel]
                float y2 = stringToNum<float>(temp_sep[9]);     // bottom[pixel]
                float h = stringToNum<float>(temp_sep[10]);     // h[m]
                float w = stringToNum<float>(temp_sep[11]);     // w[m]
                float l = stringToNum<float>(temp_sep[12]);     // l[m]
                float x = stringToNum<float>(temp_sep[13]);     // x[m]
                float y = stringToNum<float>(temp_sep[14]);     // y[m]
                float z = stringToNum<float>(temp_sep[15]);     // z[m]
                float yaw = stringToNum<float>(temp_sep[16]);   // yaw angle[rad]

                if (truncation > max_truncation || occlusion > max_occlusion \ 
                ||_trackclass[classname2id[type]]==false)
                    continue;
                cout<<"typec"<<type<<classname2id[type]<<_trackclass[classname2id[type]]<<endl;
                Eigen::Vector3d cpoint;
                cpoint << x, y, z;
                //std::cout << "came " << cpoint << "\n" << endl;

                Eigen::Vector3d ppoint = camera2cloud(cpoint);
                //std::cout << "cloud: " << ppoint << "\n" << endl;

                Detect det;
                det.box2D.resize(4);
                det.box.resize(3);
                det.classname = classname2id[type];
                /*
                det.box2D[0] = x1;
                det.box2D[1] = y1;
                det.box2D[2] = x2;
                det.box2D[2] = y2;
                */
                det.box[0] = h; // h
                det.box[1] = w; // w
                det.box[2] = l; // l
                
                det.z = ppoint(2);
                det.yaw = yaw;
                det.position = Eigen::VectorXd(2);
                det.position << ppoint(0), ppoint(1);

                labeldata[fra].push_back(det);
            }
            return labeldata.size();
        }
    }

    Eigen::Vector3d camera2cloud(Eigen::Vector3d input)
    {
        Eigen::Matrix4d RT_velo_to_cam;
        Eigen::Matrix4d R_rect;
        RT_velo_to_cam << 7.49916597e-03, -9.99971248e-01, -8.65110297e-04, -6.71807577e-03,
            1.18652889e-02, 9.54520517e-04, -9.99910318e-01, -7.33152811e-02,
            9.99882833e-01, 7.49141178e-03, 1.18719929e-02, -2.78557062e-01,
            0, 0, 0, 1;
        R_rect << 0.99992475, 0.00975976, -0.00734152, 0,
            -0.0097913, 0.99994262, -0.00430371, 0,
            0.00729911, 0.0043753, 0.99996319, 0,
            0, 0, 0, 1;

        Eigen::Vector4d point;
        point << input(0), input(1), input(2), 1;
        Eigen::Vector4d pcloud = RT_velo_to_cam.inverse() * R_rect.inverse() * point;
        Eigen::Vector3d result;
        result << pcloud(0), pcloud(1), pcloud(2);
        return result;
    }
};

#endif
