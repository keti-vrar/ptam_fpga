// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H
//#include "VideoSource.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ptam_com/PointCloud.h>
#include <ptam_com/KeyFrame_srv.h>
#include <std_msgs/String.h>
#include <queue>

#include "GLWindow2.h"
//#include "ptam/RosNode.h"
#include "ptam/Params.h"

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/so3.h>

#define LEV_IMG_BASE 0x0
#define REG_BASE 0xF9000000

#define LEV0_IMG_BASE 0x0
#define LEV0_IMG_SPAN 0x4B000

#define LEV1_IMG_BASE 0x4B000
#define LEV1_IMG_SPAN 0x12C00 

#define LEV2_IMG_BASE 0x5DC00
#define LEV2_IMG_SPAN 0x4B00

#define LEV3_IMG_BASE 0x62700
#define LEV3_IMG_SPAN 0x12C0

#define SBI_BASE 0x639C0
#define SBI_SPAN 0x4B0

#define CORNERS_POS_BASE 0x63E80
#define CORNERS_POS_SPAN 0x4E20 // 20000 bytes

#define STATUS_REG_BASE 0xF9000000
#define STATUS_REG_SPAN 0x4

#define LEV0_CORNERS_NUM_BASE 0xF9000004
#define LEV0_CORNERS_NUM_SPAN 0x4

#define LEV1_CORNERS_NUM_BASE 0xF9000008
#define LEV1_CORNERS_NUM_SPAN 0x4

#define LEV2_CORNERS_NUM_BASE 0xF900000C
#define LEV2_CORNERS_NUM_SPAN 0x4

#define LEV3_CORNERS_NUM_BASE 0xF9000010
#define LEV3_CORNERS_NUM_SPAN 0x4

#define LEV1_OFFSET 0x4B000
#define LEV2_OFFSET 0x5DC00
#define LEV3_OFFSET 0x62700
#define LEV4_OFFSET 0x639C0
#define CORNERS_POS_OFFSET 0x63E80
#define LEV0_CORNERS_NUM_OFFSET 0x4 
#define LEV1_CORNERS_NUM_OFFSET 0x8
#define LEV2_CORNERS_NUM_OFFSET 0xC
#define LEV3_CORNERS_NUM_OFFSET 0x10


// Used for FPGA_v7.5
#define F2H_BRIDGE_BASE 0x80000000
#define F2H_BRIDGE_SPAN 311296 // 307200 + 4096(spare)

#define H2F_BRIDGE_BASE 0x90000000
#define H2F_BRIDGE_SPAN 188416 //for FPGA_v7.6 //184320 for FPGA_v7.5
// lev1: 320x240 : 76800 byte
// lev2: 160x120 : 19200 byte
// lev3: 80x60   :  4800 byte
// lev4: 40x30   :  1200 byte
// nCor: lev0~3  :    32 byte
// Corn: 20000add: 80000 byte
// Sum :          182000 byte, /4096 = 44.5xxxx
//                page_size * 45 = 184320

#define LWH2F_BRIDGE_BASE 0xF9000000
#define LWH2F_BRIDGE_SPAN 4096


class ATANCamera;
struct Map;
class MapMaker;
class Tracker;
class MapViewer;

class System
{
  typedef std::queue<sensor_msgs::Imu> ImuQueue;
  typedef std::queue<geometry_msgs::PoseWithCovarianceStamped> PoseQueue;

public:
  System();
  void Run();

private:
  ros::NodeHandle nh_, image_nh_;
  ros::Subscriber sub_imu_;
  ros::Subscriber sub_calibration_;
  ros::Subscriber sub_kb_input_;
  tf::TransformBroadcaster tf_pub_;
  tf::TransformListener tf_sub_;
  image_transport::Subscriber sub_image_;
  image_transport::Publisher pub_preview_image_;
  ros::Publisher pub_pose_;             // world in the camera frame
  ros::Publisher pub_pose_world_;       // camera in the world frame
  ros::Publisher pub_info_;
  ros::ServiceServer srvPC_;
  ros::ServiceServer srvKF_;

  ros::CallbackQueue image_queue_;

  ImuQueue imu_msgs_;

  bool first_frame_;

  GLWindow2 *mGLWindow;
  MapViewer *mpMapViewer;

  CVD::Image<CVD::byte > img_bw_;
  CVD::Image<CVD::Rgb<CVD::byte> > img_rgb_;

  Map *mpMap; 
  MapMaker *mpMapMaker; 
  Tracker *mpTracker; 
  ATANCamera *mpCamera;

//  bool mbDone;

  void init(const CVD::ImageRef & size);

  void publishPoseAndInfo(const std_msgs::Header & header);
  void publishPreviewImage(CVD::Image<CVD::byte> & img, const std_msgs::Header & header);
  bool pointcloudservice(ptam_com::PointCloudRequest & req, ptam_com::PointCloudResponse & resp);
  bool keyframesservice(ptam_com::KeyFrame_srvRequest & req, ptam_com::KeyFrame_srvResponse & resp);

  void imageCallback(const sensor_msgs::ImageConstPtr & img);
  void imuCallback(const sensor_msgs::ImuConstPtr & msg);
  void keyboardCallback(const std_msgs::StringConstPtr & kb_input);

  bool transformQuaternion(const std::string & target_frame, const std_msgs::Header & header, const geometry_msgs::Quaternion & q_in, TooN::SO3<double> & r_out);
  bool transformPoint(const std::string & target_frame, const std_msgs::Header & header, const geometry_msgs::Point & t_in, TooN::Vector<3> & t_out);
  void quaternionToRotationMatrix(const geometry_msgs::Quaternion & q, TooN::SO3<double> & R);

  /// finds object in queue with timestamp closest to timestamp. Requires that T has a std_msgs::header field named "header"
  template<class T> bool findClosest(const ros::Time & timestamp, std::queue<T> & queue, T * obj, const double & max_delay = 0.01);

  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);

};



#endif
