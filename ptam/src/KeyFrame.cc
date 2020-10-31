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

using namespace CVD;
using namespace std;

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


float timediff_msec(struct timeval t0, struct timeval t1) {
	return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f; 
}

void KeyFrame::MakeKeyFrame_Lite(BasicImage<CVD::byte> &im)
{
   cout << "MakeKeyFrame_Lite+++" << endl;
   // Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
   // Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
   // e.g. does not perform FAST nonmax suppression. Things like that which are needed by the
   // mapmaker but not the tracker go in MakeKeyFrame_Rest();

   // adaptive thresholds
   static short thrs[4]={0,0,0,0};
   double buff;
   //struct timeval t0, t1, t2, t3, t4, t5, t6, t7;
   //float elapsed;
   ImageRef pos;
/*
   unsigned char* img1_ptr = new unsigned char[320*240];
   unsigned char* img2_ptr = new unsigned char[160*120];
   unsigned char* img3_ptr = new unsigned char[80*60];
   unsigned int* status_reg_ptr = new unsigned int;
   unsigned int* lev0_corners_num_ptr = new unsigned int;
   unsigned int* lev1_corners_num_ptr = new unsigned int;
   unsigned int* lev2_corners_num_ptr = new unsigned int;
   unsigned int* lev3_corners_num_ptr = new unsigned int;
   unsigned int* corners_pos_ptr = new unsigned int[15000];

   img0_ptr = (unsigned char*)lev0_img_map;
   img1_ptr = (unsigned char*)lev1_img_map;
   img2_ptr = (unsigned char*)lev2_img_map;
   img3_ptr = (unsigned char*)lev3_img_map;
   status_reg_ptr = (unsigned int*)status_reg_map;
   lev0_corners_num_ptr = (unsigned int*)lev0_corners_num_map;
   lev1_corners_num_ptr = (unsigned int*)lev1_corners_num_map;
   lev2_corners_num_ptr = (unsigned int*)lev2_corners_num_map;
   lev3_corners_num_ptr = (unsigned int*)lev3_corners_num_map;
   corners_pos_ptr = (unsigned int*)corners_pos_map;
*/

   // First, copy out the image data to the pyramid's zero level.
   cout << "im.resize()" << endl;
   aLevels[0].im.resize(im.size());

   cout << "copy im to aLevels[0].im" << endl;
   copy(im, aLevels[0].im);

#if 1
   //memset(lev0_img_ptr, 0, 307200);
   memcpy(lev0_img_ptr, im.begin(), im.totalsize());
   printf("copied im to lev0_img_ptr\n");
   //lev0_image_base = &im;
#else   
   for (int y = 0; y < im.size().y; y++) {
      for (int x = 0; x < im.size().x; x++) {
         //pos.x = x; pos.y = y;
         //*(lev0_img_ptr + (y*im.size().x+x)) = im[pos]; // assert unsigned char into volatile pointer
         *(lev0_img_ptr + (y * im.size().x + x)) = im[y][x]; 
      }
   }
   cout << "copy im to mmap()'ed region" << endl;
#endif

   cout << "Set bit" << endl;
   *(status_reg_ptr) = 0x1; // processor write is over

   while (true) {
      if (*(status_reg_ptr) == 0x3) {
  	cout << "status: ok" << endl; 
	break;
      }
      printf("waiting: %d\r", *(status_reg_ptr));
      usleep(5000);
   }
   
   //usleep(5000);
   
   cout << "Clear bit" << endl;
   *(status_reg_ptr) = 0x0; // Clear status_register
   
   //usleep(5000);

   //cout << "copy the processed data back" << endl;

   for (int i = 0; i < LEVELS; i++) // To handle SBI into Keyframe
   {
      Level &lev = aLevels[i];
      
      if (i != 0) {
         lev.im.resize(aLevels[i-1].im.size() / 2);       // image resize
#if 0
         for (int y=0; y< lev.im.size().y; y++) {
            for (int x=0; x< lev.im.size().x; x++) {
               pos.x = x; pos.y = y;
               if ( i == 1) {
                  lev.im[pos] = *(lev1_img_map + (y*lev.im.size().x+x));
               } else if ( i == 2) {
                  lev.im[pos] = *(lev2_img_map + (y*lev.im.size().x+x));
               } else if ( i == 3) {
                  lev.im[pos] = *(lev3_img_map + (y*lev.im.size().x+x));
               } 
//               else if ( i == 4) {
//                lev.im[pos] = *(lev4_image_base + (y*lev.im.size().x+x));
//             }
            }
         }
#else
	//copy(BasicImage<byte>(levels_image[i], lev.im.size()), lev.im);
        if (i == 1) { 
           copy(BasicImage<byte>(lev1_img_ptr, lev.im.size()), lev.im);
        } else if (i == 2) {
           copy(BasicImage<byte>(lev2_img_ptr, lev.im.size()), lev.im);
        } else if (i == 3) {
           copy(BasicImage<byte>(lev3_img_ptr, lev.im.size()), lev.im);
        }
#endif
      }
      //if (i == 4) { // Don't process since lev.4 is requird only for SBI input.
         //printf("small image copied\n"); 
      //   break;
      //}

      //gettimeofday(&t0, NULL);
      lev.vCorners.clear();
      lev.vCandidates.clear();
      lev.vMaxCorners.clear();

      unsigned int * result = 0; // = addr_corners[i];
      unsigned int num = 0; // = *(number_corners[i]); 
      
      if (i == 0) {
         result = corners_pos_ptr;
         num = *(lev0_corners_num_ptr);
      } else if (i == 1) {
         result = corners_pos_ptr + *(lev0_corners_num_ptr); 
         num = *(lev1_corners_num_ptr);
      } else if (i == 2) {
         result = corners_pos_ptr + *(lev0_corners_num_ptr) + *(lev1_corners_num_ptr);
         num = *(lev2_corners_num_ptr);
      } else if (i == 3) {
         result = corners_pos_ptr + *(lev0_corners_num_ptr) + *(lev1_corners_num_ptr) + *(lev2_corners_num_ptr);
         num = *(lev3_corners_num_ptr);
      } else {
         // do something  
      }

      //vector<ImageRef> levCorners;
      //levCorners.resize(num); // testing for heap corruption
      ImageRef corner_pos;
      for (unsigned int el = 0; el < num; el++) {
         // Upper 2-bytes mean X-position, Lower 2-bytes mean Y-position.
         corner_pos.x = *(result + el) >> 16;
         corner_pos.y = *(result + el) & 0x0000FFFF;
         //levCorners.push_back(corner_pos);
         lev.vCorners.push_back(corner_pos);
         //printf("result+el: 0x%x, x: 0x%x, y: 0x%x\n", *(result + el), corner_pos.x, corner_pos.y);
      }
      //printf("number_corners[%d]'size: %d\n", i, *(number_corners[i]));

      // Assign results
      //lev.vCorners = levCorners;
      cout << "lev[" << i << "] " << lev.vCorners.size() << endl;

      //cout << lev.vCorners.at(0) << lev.vCorners.at(1) << lev.vCorners.at(2) << lev.vCorners.at(3) << endl;

      const ptam::PtamParamsConfig& pPars = PtamParameters::varparams();
      if (pPars.AdaptiveThrs) {
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

   } //end of for-loop
/*
  free(img0_ptr); 
   free(img1_ptr);
   free(img2_ptr);
   free(img3_ptr);
   free(status_reg_ptr);
   free(lev0_corners_num_ptr);
   free(lev1_corners_num_ptr);
   free(lev2_corners_num_ptr);
   free(lev3_corners_num_ptr);
   free(corners_pos_ptr);
*/
   cout << "MakeKeyFrame_Lite---" << endl;
}


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
  for(int l=startlvl; l<LEVELS; l++)
  {
    Level &lev = aLevels[l];
    // .. find those FAST corners which are maximal..
    
    // minho - Why occured error.. {
    try {
      fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
    } catch (exception& e) {
      cerr << "Exception caught : " << e.what() << endl;
    }
    // }

    // .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
    // a suitably high score as Candidates, i.e. points which the mapmaker will attempt
    // to make new map points out of.
    for(vector<ImageRef>::iterator i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++)
    {
      if(!lev.im.in_image_with_border(*i, 10))
        continue;
      double dSTScore = FindShiTomasiScoreAtPoint(lev.im, 3, *i);
      if(dSTScore > gvdCandidateMinSTScore)
      {
        Candidate c;
        c.irLevelPos = *i;
        c.dSTScore = dSTScore;
        lev.vCandidates.push_back(c);
      }
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





