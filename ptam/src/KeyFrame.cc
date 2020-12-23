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
   //cout << "im.resize()" << endl;
   aLevels[0].im.resize(im.size());

   copy(im, aLevels[0].im);
#if 1
   // Copy BasicImage &im to SDRAM's lev0_img_ptr
   // void * memcpy ( void * destination, const void * source, size_t num );   
   memcpy(lev0_img_ptr, im.begin(), im.totalsize());
   //printf("Copied im to lev0_img_ptr\n");
   //std::copy(im.begin(), im.totalsize(), lev0_img_ptr);
#else   
   for (int y = 0; y < im.size().y; y++) {
      for (int x = 0; x < im.size().x; x++) {
         //pos.x = x; pos.y = y;
         //*(lev0_img_ptr + (y*im.size().x+x)) = im[pos]; // assert unsigned char into volatile pointer
         //*(lev0_img_ptr + (y * im.size().x + x)) = im[y][x]; 
         //*(lev0_img_ptr + (y * im.size().x + x)) = im[y][x];
         lev0_img_ptr.at<BasicImage<byte>>(y, x) = im[y][x]; 
      }
   }
   cout << "copy im to mmap()'ed region" << endl;
#endif

   // Set the Status Register to give ownership of SDRAM to FPGA
   // cout << "SET Status_REG" << endl;

   // FPGA reset everytime due to Design defect
   //*(status_reg_ptr) = 0x80000000;
   usleep(1000);
   
   *(status_reg_ptr) = 0x1; 

   //setitimer(ITIMER_VIRTUAL, &timer, NULL);
   //timercount = 0x1;

   unsigned int status = 1;
   //unsigned int * result_tri = NULL;
   //unsigned int num_tri = 0;
   ImageRef corner_pos2;
   int waitCnt = 0;

   while (true) { //(ros::ok()) {
      //setitimer(ITIMER_VIRTUAL, &timer, NULL);
 
      if (waitCnt > 100) { 
         *(status_reg_ptr) = 0x80000000;
         usleep(1000);
         *(status_reg_ptr) = 0x0;
         break;
      }

      status = *(status_reg_ptr);      
      if (status == 0x3) { 
         //timercount = 0x0;
         //cout << ".";
         // Insert Trigger Signal when Corner Result is Zero.
         //result_tri = corners_pos_ptr + *(lev0_corners_num_ptr) + *(lev1_corners_num_ptr) + *(lev2_corners_num_ptr);
         //corner_pos2.y = (*(result_tri) >> 16) & 0x03FF;
         //corner_pos2.x = *(result_tri) & 0x0000FFFF;
 
         //if (corner_pos2.x == 0 && corner_pos2.y == 0) {
         //  *(status_reg_ptr) |= 0x10;
         //  cout << "zero output found" << endl;
         //  printf("addr: %p, result+el: 0x%x, x: %d, y: %d\n", result_tri, *(result_tri), corner_pos2.x, corner_pos2.y);
           //*(status_reg_ptr) = 0x0;
         //}
         cout << "waitCnt: " << waitCnt << endl;
         break;
      }
      printf("Reg: %d\r", status);
      waitCnt++;
      usleep(2000);
   }

   // Clear the Status Register to get ownership of SDRAM 
   // cout << "CLEAR Status_REG" << endl;
   *(status_reg_ptr) = 0x0; 

   usleep(10000);

   /*cur_result = *(corners_pos_ptr);
   if (cur_result == prev_result) {
      // data not updated
      cout << "o";
   } else {
      // new data incoming!
      cout << "x";
      //break;
   }
   prev_result = cur_result;*/

/*
   // Insert Trigger Signal when Corner Result is Zero.
   unsigned int * result_tri = NULL;
   unsigned int num_tri = 0;

   result_tri = corners_pos_ptr + *(lev0_corners_num_ptr) + *(lev1_corners_num_ptr) + *(lev2_corners_num_ptr);
   //num_tri = *(lev3_corners_num_ptr);

   ImageRef corner_pos2;
   //for (unsigned int el = 0; el < num_tri; el++) {
   corner_pos2.y = (*(result_tri) >> 16) & 0x03FF;
   corner_pos2.x = *(result_tri) & 0x0000FFFF;
       
   if (corner_pos2.x == 0 && corner_pos2.y == 0) {
      // Do trigger for FPGA to capture output via SignalTap.
      *(status_reg_ptr) |= 0x10;
      cout << "zero output found" << endl;
      printf("addr: %p, result+el: 0x%x, x: %d, y: %d\n", result_tri, *(result_tri), corner_pos2.x, corner_pos2.y);
      *(status_reg_ptr) = 0x0;
   }
*/   

   for (int i = 0; i < LEVELS; i++) // To handle SBI into Keyframe
   {
      Level &lev = aLevels[i];      

      if (i != 0) {
         lev.im.resize(aLevels[i-1].im.size() / 2); // image resize
#if 0
         for (int y=0; y< lev.im.size().y; y++) {
            for (int x=0; x< lev.im.size().x; x++) {
               pos.x = x; pos.y = y;
               if ( i == 1) {
                  lev.im[pos] = *(lev1_img_ptr + (y*lev.im.size().x+x));
               } else if ( i == 2) {
                  lev.im[pos] = *(lev2_img_ptr + (y*lev.im.size().x+x));
               } else if ( i == 3) {
                  lev.im[pos] = *(lev3_img_ptr + (y*lev.im.size().x+x));
               } 
//               else if ( i == 4) {
//                lev.im[pos] = *(lev4_image_base + (y*lev.im.size().x+x));
//             }
            }
         }
#else
        // copy(BasicImage<byte>(levels_image[i], lev.im.size()), lev.im);
        // construct unsigned char* to BasicImage object
        // CVD::BasicImage<CVD::byte> img_sdram((CVD::byte *)lev0_img_ptr, CVD::ImageRef(lev.im.width, lev.im.height));
        if (i == 1) { 
           CVD::copy(CVD::BasicImage<CVD::byte>(lev1_img_ptr, lev.im.size()), lev.im);
        } else if (i == 2) {
           CVD::copy(CVD::BasicImage<CVD::byte>(lev2_img_ptr, lev.im.size()), lev.im);
        } else if (i == 3) {
           CVD::copy(CVD::BasicImage<CVD::byte>(lev3_img_ptr, lev.im.size()), lev.im);
        }
#endif
      }
      //if (i == 4) { // Don't process since lev.4 is requird only for SBI input.
         //printf("small image copied\n"); 
      //   break;
      //}
      //ros::Duration(0, 1000000).sleep(); // delay for 1ms
     
      lev.vCorners.clear();
      lev.vCandidates.clear();
      lev.vMaxCorners.clear();
    
      //usleep(1000);

      unsigned int num = 0;   
      unsigned int * result = NULL; 
   
      if (i == 0) {
         num = *(lev0_corners_num_ptr);
         result = corners_pos_ptr;
      } else if (i == 1) {
         num = *(lev1_corners_num_ptr);
         result = corners_pos_ptr + *(lev0_corners_num_ptr); 
      } else if (i == 2) {
         num = *(lev2_corners_num_ptr);
         result = corners_pos_ptr + *(lev0_corners_num_ptr) + *(lev1_corners_num_ptr);
      } else if (i == 3) {
         num = *(lev3_corners_num_ptr);
         result = corners_pos_ptr + *(lev0_corners_num_ptr) + *(lev1_corners_num_ptr) + *(lev2_corners_num_ptr);
      } else {
         //   
      }
      //cout << "lev: " << i << ", cn-Num: " << num << ", result_addr " << result << endl;
      cout << "lev: " << i << ", NumOfCorner: " << num << ", AddrOfCorner: " << result << endl;
#if 0
      //vector<ImageRef> levCorners;
      //levCorners.resize(num); // testing for heap corruption
      int zeroCnt = 0;
      ImageRef corner_pos;
      for (unsigned int el = 0; el < num; el++) {
         // Upper 2-bytes mean X-position, Lower 2-bytes mean Y-position.
         corner_pos.y = (*(result + el) >> 16) & 0x03FF;
         corner_pos.x = *(result + el) & 0x0000FFFF;
         //levCorners.push_back(corner_pos);
         lev.vCorners.push_back(corner_pos);
         //printf("addr: %p, result+el: 0x%x, x: %d, y: %d\n", result+el, *(result + el), corner_pos.x, corner_pos.y);
         //cout << "lev: " << i << ", cn-Num: " << num << endl;
      
         if (corner_pos.x == 0 && corner_pos.y == 0) {
         //   zeroCnt++;
         //}
         //if (zeroCnt == 5) {
            // Do trigger for FPGA to capture output via SignalTap.
            *(status_reg_ptr) |= 0x10;
            cout << "zero output found" << endl;
            printf("addr: %p, result+el: 0x%x, x: %d, y: %d\n", result+el, *(result     + el), corner_pos.x, corner_pos.y);
            cout << "lev: " << i << ", cn-Num: " << num << endl;

            *(status_reg_ptr) = 0x0;
         }
      }
#else 
      //usleep(10000);

      a = new unsigned int[num];
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
/*
         if (corner_pos.x == 0 && corner_pos.y == 0) {
             // Do trigger for FPGA to capture output via SignalTap.
             *(status_reg_ptr) |= 0x10;
             
             printf("CornerOfRaw: 0x%x, x: %d, y: %d\n", tmp, corner_pos.x, corner_pos.y);
             cout << "lev: " << i << ", CornerSize: " << num << "<---Zero Corner detected " << endl;
 
             *(status_reg_ptr) = 0x0;
         } 
        
         if (i == 0) { 
            if (corner_pos.x > 640 || corner_pos.y > 480) {
                cout << "ERROR !!! lev: " << i << ", [" << el << "/" << num << "] has x, y: " << corner_pos.x << ", " << corner_pos.y << endl;
            }
         } else if (i == 1) {
            if (corner_pos.x > 320 || corner_pos.y > 240) {
               cout << "ERROR !!! lev: " << i << ", [" << el << "/" << num << "] has x, y: " << corner_pos.x << ", " << corner_pos.y << endl;
            }
         } else if (i == 2) {
            if (corner_pos.x > 160 || corner_pos.y > 120) {
               cout << "ERROR !!! lev: " << i << ", [" << el << "/" << num << "] has x, y: " << corner_pos.x << ", " << corner_pos.y << endl;
            }  
         } else if (i == 3) {
            if (corner_pos.x > 80 || corner_pos.y > 60) {
               cout << "ERROR !!! lev: " << i << ", [" << el << "/" << num << "] has x, y: " << corner_pos.x << ", " << corner_pos.y << endl;
            }
         }
*/
      }
#endif
      //printf("lev[%d], cnNum-Reg: %d, cnSize-Vec: %d\n", i, num, lev.vCorners.size());//.*(number_corners[i]));
      //if (i != 0) {
      /*for (int cn = 0; cn < lev.vCorners.size(); cn++) {
         cout << "lev[" << i << "]" << "[" << cn << "/" << lev.vCorners.size() << "] " << lev.vCorners.at(cn) << endl; 
      } */
      //}
      // Assign results
      //cout << "lev[" << i << "] " << lev.vCorners.size() << endl;
      //cout << lev.vCorners.at(0) << lev.vCorners.at(1) << lev.vCorners.at(2) << lev.vCorners.at(3) << endl;
      int same = 0;
      if (num == lev.vCorners.size()) {
         same = 1;
      } 
      cout << "CN-NumReg: " << num << ", CN-Vec: " << lev.vCorners.size() << "?=" << same << endl;

 
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
#if 0
      for(int y=0; y<lev.im.size().y; y++)
      {
         while(v < lev.vCorners.size() && y > lev.vCorners[v].y)
            v++;
            lev.vCornerRowLUT.push_back(v);
      }
#else
      for(int y=0; y<lev.im.size().y; y=y+2)
      { 
         while (v < lev.vCorners.size() && y > lev.vCorners[v].y)
           v++;

         lev.vCornerRowLUT.push_back(v);
         lev.vCornerRowLUT.push_back(v);
      }
#endif
      delete [] a;
      a = NULL;

   }; //end of for-loop

}  // end of MakeKeyFrame_Lite()


#if 0 
   unsigned int status = 0;
   while(1) // polling...
   {
      status = *(status_register);
      //printf("Waiting for FPGA completed... Checking Status: %d\r", status); 
      //printf(".");

      // If status is 0, GET the halfsampled image data and corner detection result
      if (status != 1)
      {
         //printf("status : %d, frame_cnt : %d\n", status, frame_cnt);
         //printf("\n");

         for (int i = 0; i < LEVELS+1; i++) // To handle SBI into Keyframe
         {
            Level &lev = aLevels[i];

            if (i != 0)
            { 
               lev.im.resize(aLevels[i-1].im.size() / 2);       // image resize

               for (int y=0; y< lev.im.size().y; y++) {
                  for (int x=0; x< lev.im.size().x; x++) {
                     pos.x = x; pos.y = y;
                     if ( i == 1) {
                        lev.im[pos] = *(lev1_image_base + (y*lev.im.size().x+x));
                     } else if ( i == 2) {
                        lev.im[pos] = *(lev2_image_base + (y*lev.im.size().x+x));
                     } else if ( i == 3) {
                        lev.im[pos] = *(lev3_image_base + (y*lev.im.size().x+x));
                     } else if ( i == 4) {
                        lev.im[pos] = *(lev4_image_base + (y*lev.im.size().x+x));
                     }
                  }
               }

            }
            if (i == 4) { // Don't process since lev.4 is requird only for SBI input.
               //printf("small image copied\n"); 
               break;
            }

            //gettimeofday(&t0, NULL);
	    lev.vCorners.clear();
            lev.vCandidates.clear();
            lev.vMaxCorners.clear();

            volatile unsigned int* result = addr_corners[i];
            vector<ImageRef> levCorners;
            for (int el = 0; el < *(number_corners[i]); el++) {
               // Upper 2-bytes mean X-position, Lower 2-bytes mean Y-position.
               ImageRef corner_pos;
               corner_pos.x = *(result + el) >> 16;
               corner_pos.y = *(result + el) & 0x0000FFFF;
               levCorners.push_back(corner_pos);
               //printf("result+el: 0x%x, x: 0x%x, y: 0x%x\n", *(result + el), corner_pos.x, corner_pos.y);
            }
            printf("number_corners[%d]'size: %d\n", i, *(number_corners[i]));
 
            // Assign results
            lev.vCorners = levCorners;
            // printf("[F %d][LEVEL %d] Corner detection results size : %d\n", frame_cnt, i, lev.vCorners.size());
            //gettimeofday(&t1, NULL);
            //elapsed = timediff_msec(t0, t1);
            //printf("Asserting Corners' duration: %f ms\n", elapsed);
                        
            const ptam::PtamParamsConfig& pPars = PtamParameters::varparams();
            if (pPars.AdaptiveThrs)
            {
               buff = lev.vCorners.size()-pPars.AdaptiveThrsMult*pPars.MaxPatchesPerFrame/pow(2.0,i);
               thrs[i] = thrs[i]+(buff>0)-(buff<0);
            }
            else 
               thrs[i]=0;

           // Generate row look-up-table for the FAST corner points: this speeds up
           // finding close-by corner points later on.
           //gettimeofday(&t2, NULL);

           unsigned int v=0;
           lev.vCornerRowLUT.clear();
           for(int y=0; y<lev.im.size().y; y++)
           {
              while(v < lev.vCorners.size() && y > lev.vCorners[v].y)
                 v++;
              lev.vCornerRowLUT.push_back(v);
           }
           //printf("1st stage done\n");
           //gettimeofday(&t3, NULL);
           //elapsed = timediff_msec(t2, t3);
           //printf("Asserting RowLUT's duration: %f ms\n", elapsed); 
         } //end of for-loop 
      //break;
      } // end of status checking if (status != 1)
      break; 
   } // end of while loop 
  *(status_register) = 0x0;
  //munmap(hps_virtual_base, SDRAM_SPAN);
  //munmap(lw_axi_h2f_base, AXI_LW_MEM_SPAN);
  cout << "MakeKeyFrame_Lite---" << endl;
  // } minho
}
#endif

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
         cout << "!lev.im.in_image_wih_border" << endl;
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
      cout << "lev.vCandidates.size: " << lev.vCandidates.size() << endl;
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


