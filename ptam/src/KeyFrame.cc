// Copyright 2008 Isis Innovation Limited
#include "ptam/KeyFrame.h"
#include "ptam/ShiTomasi.h"
#include "ptam/SmallBlurryImage.h"
#include <cvd/vision.h>
#include <cvd/fast_corner.h>
#include <agast/agast_corner_detect.h>
//{slynen reprojection
#include "ptam/MapPoint.h"
#include "ptam/TrackerData.h"
//}

#include <sys/mman.h>
#include <time.h>
#include "ros/time.h"
//#include "ros/ros.h"

#include "ptam/System.h"

#include <signal.h>
#include <string.h>
#include <sys/time.h>

//#include <chrono>
//#include <thread>

// To save CVD image into file
//#include <cvd/image_io.h>
//#include <sstream>

using namespace CVD;
using namespace std;

//extern void* lev_img_map;
//extern void* reg_map;
extern void* f2h_virtual_base;
extern void* h2f_virtual_base;
extern void* lwh2f_virtual_base;

extern unsigned char* lev0_img_ptr;
extern unsigned char* lev1_img_ptr;
extern unsigned char* lev2_img_ptr;
extern unsigned char* lev3_img_ptr;

extern unsigned int* corners_pos_ptr;
extern unsigned int* status_reg_ptr;
extern unsigned int* lev0_corners_num_ptr;
extern unsigned int* lev1_corners_num_ptr;
extern unsigned int* lev2_corners_num_ptr;
extern unsigned int* lev3_corners_num_ptr;

//extern int length_lev;
//extern int length_reg;

//static std::stringstream initstr;
int framecnt = 0;
//unsigned int raw_corners[10000] = {0,};
static unsigned int prev_result = 0;
static unsigned int cur_result = 0;

int timercount = 0;
extern int mapOK;
extern int badCorner;
//#define lev0_length 307200

/*
float timediff_msec(struct timeval t0, struct timeval t1) {
	return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f; 
}
*/

//void callback1(const ros::TimerEvent&) {
//   ROS_INFO("Callback 1 triggered");
//}
/*
void timer_handler(int signum) {
   //static int count = 0;
   //cout << "timer expired " << timercount << " timers" << endl;
   //cout << "timer callback called" << endl;
 
   // Trigger via Status_Register bit5
   if (timercount == 1) {
      *(status_reg_ptr) |= 0x10;
      cout << "timer expired " << timercount << " timers" << endl;
      *(status_reg_ptr) = 0x0;
   }

   timercount = 0x0;
}
*/
void KeyFrame::MakeKeyFrame_Lite(BasicImage<CVD::byte> &im)
{
   //cout << "MakeKeyFrame_Lite+++" << endl;

   // Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
   // Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
   // e.g. does not perform FAST nonmax suppression. Things like that which are needed by the
   // mapmaker but not the tracker go in MakeKeyFrame_Rest();

   // adaptive thresholds
   static short thrs[4]={0,0,0,0};
   double buff;
   ImageRef pos;
   clock_t t;
   unsigned int* a = NULL;
  /* 
   struct sigaction sa;
   struct itimerval timer;
   memset(&sa, 0, sizeof(sa));
   sa.sa_handler = &timer_handler;
   sigaction(SIGVTALRM, &sa, NULL);

   timer.it_value.tv_sec = 0;
   timer.it_value.tv_usec = 8000;

   timer.it_interval.tv_sec = 0;
   timer.it_interval.tv_usec = 8000;
  */
   framecnt++;
   if (framecnt % 10 == 0) {
      t = clock();
      printf("%f, %d\n", ((float)t)/CLOCKS_PER_SEC, framecnt);
   }
   
   // First, copy out the image data to the pyramid's zero level.
   aLevels[0].im.resize(im.size());

   copy(im, aLevels[0].im);

   if (mapOK == 0) { // SW mode
        // Then, for each level...
	for(int i=0; i<LEVELS; i++)
	{
		Level &lev = aLevels[i];
		if(i!=0)
		{  // .. make a half-size image from the previous level..
			lev.im.resize(aLevels[i-1].im.size() / 2);
			halfSample(aLevels[i-1].im, lev.im);
		}

		lev.vCorners.clear();
		lev.vCandidates.clear();
		lev.vMaxCorners.clear();

		void (*pFASTFunc)(const CVD::BasicImage<CVD::byte> &, std::vector<CVD::ImageRef> &,int)=NULL;
		
		const ptam::PtamParamsConfig& pPars = PtamParameters::varparams();
		pFASTFunc=&fast_corner_detect_9_nonmax;
		if (pPars.FASTMethod=="FAST9")
			pFASTFunc=&fast_corner_detect_9;
		else if (pPars.FASTMethod=="FAST10")
			pFASTFunc=&fast_corner_detect_10;
		else if (pPars.FASTMethod=="FAST9_nonmax")
			pFASTFunc=&fast_corner_detect_9_nonmax;
		else if (pPars.FASTMethod=="AGAST12d")
			pFASTFunc=&agast::agast7_12d::agast_corner_detect12d;
		else if (pPars.FASTMethod=="OAST16")
			pFASTFunc=&agast::oast9_16::oast_corner_detect16;

		if(i == 0)
			pFASTFunc(lev.im, lev.vCorners, pPars.Thres_lvl0+thrs[i]);
		if(i == 1)
			pFASTFunc(lev.im, lev.vCorners, pPars.Thres_lvl1+thrs[i]);
		if(i == 2)
			pFASTFunc(lev.im, lev.vCorners, pPars.Thres_lvl2+thrs[i]);
		if(i == 3)
			pFASTFunc(lev.im, lev.vCorners, pPars.Thres_lvl3+thrs[i]);

	/*	for (int j = 0; j < lev.vCorners.size(); j++) {
			cout << "lev[" << i << "], num [" << j << "]" << lev.vCorners.at(j) << endl;
		} */

		if (pPars.AdaptiveThrs)
		{
			buff = lev.vCorners.size()-pPars.AdaptiveThrsMult*pPars.MaxPatchesPerFrame/pow(2.0,i);
			thrs[i] = thrs[i]+(buff>0)-(buff<0);
		}
		else
			thrs[i]=0;

		unsigned int v=0;
		lev.vCornerRowLUT.clear();
		for(int y=0; y<lev.im.size().y; y++)
		{
			while(v < lev.vCorners.size() && y > lev.vCorners[v].y)
				v++;
			lev.vCornerRowLUT.push_back(v);
		}
	}; // end of for-loop

   } else { // FPGA mode
        try {
           if (lev0_img_ptr != NULL) {
              memcpy(lev0_img_ptr, im.begin(), im.totalsize());
            } else {
              return;
            }
        } catch (exception& exp) {
           cout << "Exception: " << exp.what() << endl;
        }
      
        usleep(1000);
        *(status_reg_ptr) = 0x1;

        unsigned int status = 1;
        int waitCnt = 0;
        unsigned int corNum_lv0 = 0;
        unsigned int corNum_lv1 = 0;
        unsigned int corNum_lv2 = 0;
        unsigned int corNum_lv3 = 0;
        unsigned int corner_result = 0;

        while (true) { 
           if (waitCnt > 4000) { //5) {
              *(status_reg_ptr) = 0x80000000;
              usleep(1000);
              *(status_reg_ptr) = 0x0;
              break;
           }
           status = *(status_reg_ptr);
           if (status == 0x3) {
              // TODO
              // Add Trigger when detected invalid corner number
              //usleep(500);
               
              corNum_lv0 = *(lev0_corners_num_ptr);
              if (corNum_lv0 > 34133) {
                 // Trigger the signal tap
                 corNum_lv1 = *(lev1_corners_num_ptr);
                 corNum_lv2 = *(lev2_corners_num_ptr);
                 corNum_lv3 = *(lev3_corners_num_ptr);
                 corner_result = *(corners_pos_ptr);

                 *(status_reg_ptr) |= 0x10;

                 /*cout << "<<< SignalTap Triggered >>>" << endl;
                 cout << "corNum_lv0: " << corNum_lv0 << ", corNum_lv1: " << corNum_lv1 << ", corNum_lv2: " << corNum_lv2 << ", corNum_lv3: " << corNum_lv3 << endl;
                 cout << "corner_result: " << corner_result << endl;
                 */
                 *(status_reg_ptr) = 0x0;
                 
                 badCorner = 0x1;
                 //cout << "return from MakeKeyFrame_Lite" << endl;
                 *(status_reg_ptr) = 0x80000000;
                 usleep(1000);
                 *(status_reg_ptr) = 0x0;
                 return;
              } 
              //cout << "waitCnt: " << waitCnt << endl;
              break;
           }
           //printf("Reg: %d\r", status);
           printf(".\r");
           waitCnt++;
           //usleep(2000);
        }

        *(status_reg_ptr) = 0x0;
        usleep(10000);
     
        for (int i = 0; i < LEVELS; i++) // To handle SBI into Keyframe
        {
           Level &lev = aLevels[i];
 
           if (i != 0) {
              lev.im.resize(aLevels[i-1].im.size() / 2); // image resize

              if (i == 1) {
                 CVD::copy(CVD::BasicImage<CVD::byte>(lev1_img_ptr, lev.im.size()), lev.im);
              } else if (i == 2) {
                 CVD::copy(CVD::BasicImage<CVD::byte>(lev2_img_ptr, lev.im.size()), lev.im);
              } else if (i == 3) {
                 CVD::copy(CVD::BasicImage<CVD::byte>(lev3_img_ptr, lev.im.size()), lev.im);
              }
           }

           lev.vCorners.clear();
           lev.vCandidates.clear();
           lev.vMaxCorners.clear();
 
           unsigned int num = 0;
           unsigned int * result = NULL;
 
           if (i == 0) {
              num = *(lev0_corners_num_ptr);
              if (num > 307200) {
                 //usleep(10000);
                 cout << "invalid (lev0)corner number detected : " << num << endl;
                 //num = *(lev0_corners_num_ptr);
                 badCorner = 0x1;
                 return;
              } 
              result = corners_pos_ptr;
           } else if (i == 1) {
              num = *(lev1_corners_num_ptr);
              if (num > 76800) {
                  //usleep(10000);
                  cout << "invalid (lev1)corner number detected : " << num << endl;
                  //num = *(lev1_corners_num_ptr);
                  badCorner = 0x1;
                  return;
              }
              result = corners_pos_ptr + *(lev0_corners_num_ptr);
           } else if (i == 2) {
              num = *(lev2_corners_num_ptr);
              if (num > 19200) {
                 //usleep(10000);
                 cout << "invalid (lev2)corner number detected : " << num << endl;
                 //num = *(lev2_corners_num_ptr);
                 badCorner = 0x1;
                 return;
              }
              result = corners_pos_ptr + *(lev0_corners_num_ptr) + *(lev1_corners_num_ptr);
           } else if (i == 3) {
              num = *(lev3_corners_num_ptr);
              if (num > 4800) {
                 //usleep(10000);
                 cout << "invalid (lev3)corner number detected : " << num << endl;
                 //num = *(lev3_corners_num_ptr);
                 badCorner = 0x1;
                 return;
              }
              result = corners_pos_ptr + *(lev0_corners_num_ptr) + *(lev1_corners_num_ptr) + *(lev2_corners_num_ptr);
           } else {
              //   
           }
           badCorner = 0x0;
           //cout << "lev: " << i << ", NumOfCorner: " << num << ", AddrOfCorner: " << result << endl;

           try {
              a = new unsigned int[num];
           } catch (bad_alloc& bad) {
              cout << "Not enough memory, num: " << num << endl;
              return;
           }
 
           if (a != NULL || result != NULL) {
              std::fill_n(a, num, 0);
              std::copy(result, result+num, a);
 
              ImageRef corner_pos;
              unsigned int tmp = 0;
              for (unsigned int el=0; el < num; el++) {
                 tmp = *(a + el);
 
                 corner_pos.y = (tmp >> 16) & 0x3FF;
                 corner_pos.x = tmp & 0x0000FFFF;
 
                 if (i == 0 && (corner_pos.x > 640 || corner_pos.y > 480)) {
                    continue;
                 } else if (i == 1 && (corner_pos.x > 320 || corner_pos.y > 240)) {
                    continue;
                 } else if (i == 2 && (corner_pos.x > 160 || corner_pos.y > 120)) {
                    continue;
                 } else if (i == 3 && (corner_pos.x > 80 || corner_pos.y > 60)) {
                    continue;
                 }
                 lev.vCorners.push_back(corner_pos);
              }
           }
           else {
              // access mmap()ed area when failed array for corner result in heap 
              return;
           }
         
           int same = 0;
           if (num == lev.vCorners.size()) {
               same = 1;
           }
           //cout << "CN-NumReg: " << num << ", CN-Vec: " << lev.vCorners.size() << "?=" << same << endl;
 
           const ptam::PtamParamsConfig& pPars = PtamParameters::varparams();
           if (pPars.AdaptiveThrs) {
              buff = lev.vCorners.size()-pPars.AdaptiveThrsMult*pPars.MaxPatchesPerFrame/pow(2.0,i);
              thrs[i] = thrs[i]+(buff>0)-(buff<0);
           }
           else
              thrs[i]=0;
 
           // Generate row look-up-table for the FAST corner points: this speeds up
           // finding close-by corner points later on.
           unsigned int v=0;
           lev.vCornerRowLUT.clear();
           for(int y=0; y<lev.im.size().y; y=y+2)
           {
              while (v < lev.vCorners.size() && y > lev.vCorners[v].y)
                v++;

              lev.vCornerRowLUT.push_back(v);
              lev.vCornerRowLUT.push_back(v);
           }
     
           delete [] a;
           a = NULL;
     
      }; // End of for-loop

   } // End of Else

} // end of MakeKeyframe_Lite()


void KeyFrame::MakeKeyFrame_Rest()
{
  // Fills the rest of the keyframe structure needed by the mapmaker:
  // FAST nonmax suppression, generation of the list of candidates for further map points,
  // creation of the relocaliser's SmallBlurryImage.
  //Weiss{
  //static double gvdCandidateMinSTScore = 70;
  
  const FixParams& pPars = PtamParameters::fixparams();
  const VarParams& pVarPars = PtamParameters::varparams();
  static double gvdCandidateMinSTScore = pPars.CandidateMinSTScore;
  //static gvar3<double> gvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, SILENT);
  //}


  // For each level...
  int startlvl=0;
  if(pVarPars.NoLevelZeroMapPoints)
  {
    startlvl=1;	// ignore level zero points for the map
  }
  //cout << "startlvl: " << startlvl << endl;

  for(int l=startlvl; l<LEVELS; l++)
  {
    Level &lev = aLevels[l];
    // .. find those FAST corners which are maximal..
    
    // minho - Why occured error.. {

    /*try {
      fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
    } catch (exception& e) {
      cerr << "Exception caught : " << e.what() << endl;
    }*/
    // }

    lev.vMaxCorners = lev.vCorners;
    // .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
    // a suitably high score as Candidates, i.e. points which the mapmaker will attempt
    // to make new map points out of.
    for(vector<ImageRef>::iterator i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++)
    {
      if(!lev.im.in_image_with_border(*i, 2)) { //5)) { //10)) {
         //cout << "!lev.im.in_image_wih_border" << endl;
         continue;
      }
      double dSTScore = FindShiTomasiScoreAtPoint(lev.im, 3, *i);
      if(dSTScore > gvdCandidateMinSTScore)
      {
        Candidate c;
        c.irLevelPos = *i;
        c.dSTScore = dSTScore;
        lev.vCandidates.push_back(c);
      }
      //cout << "lev.vCandidates.size: " << lev.vCandidates.size() << endl;
    }

  };

  // Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
  pSBI = new SmallBlurryImage(*this);
  // Relocaliser also wants the jacobians..
  pSBI->MakeJacs();

  //Weiss{
  static unsigned int idcounter=0;
  ID=idcounter;
  idcounter++;
  //}
}

//{slynen reprojection
// checks whether to add a MapPoint to the KeyMapPoint of the KeyFrame
// those points help selecting the Keyframes which are visible for reprojection
void KeyFrame::AddKeyMapPoint(MapPoint::Ptr mp){
  vpPoints.push_back(mp);
  //first point init
  if(apCurrentBestPoints[0]==NULL){
    for (unsigned int i = 0;i < iBestPointsCount; i++) apCurrentBestPoints[i] = mp;
    return;
  }
  //apCurrentBestPoints indices:center = 0;upperright = 1;upperleft = 2;lowerleft = 3;lowerright = 4
  CVD::ImageRef imgRef  = aLevels[0].im.size();

  //all point locations in level zero coords
  Vector<2> v2Imcenter = makeVector(imgRef.x / 2,imgRef.y / 2);
  Vector<2> v2PointPos = LevelZeroPos(mp->irCenter,0);
  Vector<2> v2Diff = v2PointPos - v2Imcenter;	//distance to center of level zero
  Vector<2> v2DiffBest; //best distance for this point

  v2DiffBest = LevelZeroPos(apCurrentBestPoints[0]->irCenter,apCurrentBestPoints[0]->nSourceLevel) - v2Imcenter;
  if((v2Diff*v2Diff) < (v2DiffBest*v2DiffBest))
    apCurrentBestPoints[0] = mp;	//the point is closer to the center that current best center point

  //now check which quadrant the point is in
  if(v2PointPos[0] > v2Imcenter[0] &&  v2PointPos[1] > v2Imcenter[1]){
    v2DiffBest = LevelZeroPos(apCurrentBestPoints[1]->irCenter,apCurrentBestPoints[1]->nSourceLevel) - v2Imcenter;
    if((v2Diff*v2Diff) > (v2DiffBest*v2DiffBest))
      apCurrentBestPoints[1] = mp; //further away than current best point
  }else if(v2PointPos[0]<v2Imcenter[0] &&  v2PointPos[1] > v2Imcenter[1]){
    v2DiffBest = LevelZeroPos(apCurrentBestPoints[2]->irCenter,apCurrentBestPoints[2]->nSourceLevel) - v2Imcenter;
    if((v2Diff*v2Diff) > (v2DiffBest*v2DiffBest))
      apCurrentBestPoints[2] = mp; //further away than current best point
  }else if(v2PointPos[0]<v2Imcenter[0] &&  v2PointPos[1] < v2Imcenter[1]){
    v2DiffBest = LevelZeroPos(apCurrentBestPoints[3]->irCenter,apCurrentBestPoints[3]->nSourceLevel) - v2Imcenter;
    if((v2Diff*v2Diff) > (v2DiffBest*v2DiffBest))
      apCurrentBestPoints[3] = mp; //further away than current best point
  }else if(v2PointPos[0]>v2Imcenter[0] &&  v2PointPos[1] < v2Imcenter[1]){
    v2DiffBest = LevelZeroPos(apCurrentBestPoints[4]->irCenter,apCurrentBestPoints[4]->nSourceLevel) - v2Imcenter;
    if((v2Diff*v2Diff) > (v2DiffBest*v2DiffBest))
      apCurrentBestPoints[4] = mp; //further away than current best point
  }
}
//}

// The keyframe struct is quite happy with default operator=, but Level needs its own
// to override CVD's reference-counting behaviour.
Level& Level::operator=(const Level &rhs)
{
  // Operator= should physically copy pixels, not use CVD's reference-counting image copy.
  im.resize(rhs.im.size());
  copy(rhs.im, im);

  vCorners = rhs.vCorners;
  vMaxCorners = rhs.vMaxCorners;
  vCornerRowLUT = rhs.vCornerRowLUT;
  return *this;
}

// -------------------------------------------------------------
// Some useful globals defined in LevelHelpers.h live here:
Vector<3> gavLevelColors[LEVELS];

// These globals are filled in here. A single static instance of this struct is run before main()
struct LevelHelpersFiller // Code which should be initialised on init goes here; this runs before main()
{
  LevelHelpersFiller()
  {
    for(int i=0; i<LEVELS; i++)
    {
      if(i==0)  gavLevelColors[i] = makeVector( 1.0, 0.0, 0.0);
      else if(i==1)  gavLevelColors[i] = makeVector( 1.0, 1.0, 0.0);
      else if(i==2)  gavLevelColors[i] = makeVector( 0.0, 1.0, 0.0);
      else if(i==3)  gavLevelColors[i] = makeVector( 0.0, 0.0, 0.7);
      else gavLevelColors[i] =  makeVector( 1.0, 1.0, 0.7); // In case I ever run with LEVELS > 4
    }
  }
};

static LevelHelpersFiller foo;


