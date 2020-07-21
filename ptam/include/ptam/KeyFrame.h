// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the data structures to do with keyframes:
// structs KeyFrame, Level, Measurement, Candidate.
// 
// A KeyFrame contains an image pyramid stored as array of Level;
// A KeyFrame also has associated map-point mesurements stored as a vector of Measurment;
// Each individual Level contains an image, corner points, and special corner points
// which are promoted to Candidate status (the mapmaker tries to make new map points from those.)
//
// KeyFrames are stored in the Map class and manipulated by the MapMaker.
// However, the tracker also stores its current frame as a half-populated
// KeyFrame struct.


#ifndef __KEYFRAME_H
#define __KEYFRAME_H
#include <TooN/TooN.h>
using namespace TooN;
#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <vector>
#include <set>
#include <map>
#include <boost/shared_ptr.hpp>

#include "ptam/Params.h"

#include <signal.h>
#include <boost/lexical_cast.hpp>   // int to string

struct MapPoint;
class SmallBlurryImage;
//slynen{ reprojection
struct TrackerData;
//}
#define LEVELS 4

#define SDRAM_BASE 0x0
#define SDRAM_SPAN 0xA00000  // 10MB KByte

#define AXI_LW_MEM_BASE 0xF9000000
#define AXI_LW_MEM_SPAN	0x200000 // 2MB

#define MEMORY_SIZE_LEVEL0 0x4B000 // Lv.0 image 640x480
#define MEMORY_SIZE_LEVEL1 0x12C00 // Lv.1 image 320x240
#define MEMORY_SIZE_LEVEL2 0x4B00  // Lv.2 image 160x120
#define MEMORY_SIZE_LEVEL3 0x12C0  // Lv.3 image 80x60
#define MEMORY_SIZE_LEVEL4 0x4B0   // Lv.4 image 40x30 for SBI

#define STATUS_REG_0_OFFSET     0x0
#define N_CORNERS_LEVEL0_OFFSET 0x4
#define N_CORNERS_LEVEL1_OFFSET 0x8
#define N_CORNERS_LEVEL2_OFFSET 0xC
#define N_CORNERS_LEVEL3_OFFSET 0x10


const int img_size_of_level[LEVELS+1] = {
     MEMORY_SIZE_LEVEL0,  // Lv.0 image 640x480
     MEMORY_SIZE_LEVEL1,  // Lv.1 image 320x240
     MEMORY_SIZE_LEVEL2,  // Lv.2 image 160x120
     MEMORY_SIZE_LEVEL3,  // Lv.3 image 80x60
     MEMORY_SIZE_LEVEL4   // Lv.4 image 40x30, SBI
};


// Candidate: a feature in an image which could be made into a map point
struct Candidate
{
  CVD::ImageRef irLevelPos;
  Vector<2> v2RootPos;
  double dSTScore;
};

// Measurement: A 2D image measurement of a map point. Each keyframe stores a bunch of these.
struct Measurement
{
  int nLevel;   // Which image level?
  bool bSubPix; // Has this measurement been refined to sub-pixel level?
  Vector<2> v2RootPos;  // Position of the measurement, REFERED TO PYRAMID LEVEL ZERO
  enum {SRC_TRACKER, SRC_REFIND, SRC_ROOT, SRC_TRAIL, SRC_EPIPOLAR} Source; // Where has this measurement come frome?
};

// Each keyframe is made of LEVELS pyramid levels, stored in struct Level.
// This contains image data and corner points.
struct Level
{
  inline Level()
  {
    bImplaneCornersCached = false;
  };

  CVD::Image<CVD::byte> im;                // The pyramid level pixels
  std::vector<CVD::ImageRef> vCorners;     // All FAST corners on this level
  std::vector<int> vCornerRowLUT;          // Row-index into the FAST corners, speeds up access
  std::vector<CVD::ImageRef> vMaxCorners;  // The maximal FAST corners
  Level& operator=(const Level &rhs);

  std::vector<Candidate> vCandidates;   // Potential locations of new map points

  bool bImplaneCornersCached;           // Also keep image-plane (z=1) positions of FAST corners to speed up epipolar search
  std::vector<Vector<2> > vImplaneCorners; // Corner points un-projected into z=1-plane coordinates
};

// The actual KeyFrame struct. The map contains of a bunch of these. However, the tracker uses this
// struct as well: every incoming frame is turned into a keyframe before tracking; most of these 
// are then simply discarded, but sometimes they're then just added to the map.
struct KeyFrame
{
  typedef boost::shared_ptr<KeyFrame> Ptr;

  inline KeyFrame()
  {
    pSBI = NULL;
  }
  SE3<> se3CfromW;    // The coordinate frame of this key-frame as a Camera-From-World transformation
  bool bFixed;      // Is the coordinate frame of this keyframe fixed? (only true for first KF!)
  Level aLevels[LEVELS+1];  // Images, corners, etc lives in this array of pyramid levels
  std::map<boost::shared_ptr<MapPoint>, Measurement> mMeasurements;           // All the measurements associated with the keyframe

  //slynen pcl interface{
  int bID;
  //}
  void MakeKeyFrame_Lite(CVD::BasicImage<CVD::byte> &im);   // This takes an image and calculates pyramid levels etc to fill the
  // keyframe data structures with everything that's needed by the tracker..
  void MakeKeyFrame_Rest(); // ... while this calculates the rest of the data which the mapmaker needs.

  //slynen{ reprojection
  std::vector<boost::shared_ptr<MapPoint> > vpPoints;                          // stores the map points found in this keyframe
  enum { iBestPointsCount = 5 };                    // how many points to use for keyframe evaluation
  boost::shared_ptr<MapPoint> apCurrentBestPoints[iBestPointsCount];	               // stores the currently best Points for Keyframe identification
  void AddKeyMapPoint(boost::shared_ptr<MapPoint> mp);                        // checks whether to add a MapPoint to the KeyMapPoint of the KeyFrame
  // those points help selecting the Keyframes which are visible for reprojection
  //}

  double dSceneDepthMean;      // Hacky hueristics to improve epipolar search.
  double dSceneDepthSigma;

  //Weiss{
  double dSceneDepthMedian;	// used to keep same scale after auto-re-init
  int ID;			// KF id used to identify KFs when exporting them...
  //}

  SmallBlurryImage *pSBI; // The relocaliser uses this
};

typedef std::map<boost::shared_ptr<MapPoint>, Measurement>::iterator meas_it;  // For convenience, and to work around an emacs paren-matching bug


#endif

