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

// minho {
#include <cvd/image_io.h>   // libcvd. img_save()
#include <fcntl.h>    // file open option
#include <unistd.h>   // mmap
#include <sys/mman.h> // mmap
#include <boost/lexical_cast.hpp>   // int to string
#include <signal.h>

#define IMG_SIZE_640X480 0x0004b000	// 307200 bytes
#define IMG_SIZE_320X240 0x00012C00	// 76800 bytes
#define IMG_SIZE_160X120 0x00004B00	// 19200 bytes
#define IMG_SIZE_80X60 0x000012C0	// 4800 bytes	// total 408000 (0006_39C0)
// }


// minho {
static bool is_first_frame = true;
static unsigned long int frame_cnt = 0;

//void *sdram_virtual_base;
//unsigned char *sdram_status_register = NULL;	
//unsigned char *sdram_data_ptr = NULL;
//unsigned char *sdram_data_ptr_level[LEVELS+1] = {};
//int *sdram_data_ptr_cordet = NULL;

struct sigaction sa;

void mmap_init();
void on_close(int signal);
void my_close();
void segfault_sigaction(int signal, siginfo_t *si, void *arg);
// minho }

void* hps_virtual_base;
void* lw_axi_h2f_base;

unsigned int* IMG_Lev[LEVELS+1] = {};
//CVD::Image<CVD::byte> *IMG_Lev[LEVELS+1] = {};

unsigned int* pCorners_Lev[LEVELS] = {};
unsigned int* nCorners_Lev[LEVELS] = {};

unsigned int* status_Register = NULL;	
unsigned int* cornerNumber = NULL;

using namespace CVD;
using namespace std;
//using namespace GVars3;

float timediff_msec(struct timeval t0, struct timeval t1) {
	return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f; 
}

void KeyFrame::MakeKeyFrame_Lite(BasicImage<CVD::byte> &im)
{
   // Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
   // Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
   // e.g. does not perform FAST nonmax suppression. Things like that which are needed by the
   // mapmaker but not the tracker go in MakeKeyFrame_Rest();

   // adaptive thresholds
   static short thrs[4]={0,0,0,0};
   double buff;
   struct timeval t0, t1, t2, t3, t4, t5, t6, t7;
   float elapsed;

   // minho {
   frame_cnt++;

   if (is_first_frame) 
   {
      mmap_init();
      is_first_frame = false;
   }
   // minho }
/*
   pCorners_Lev[1] = pCorners_Lev[0] + *(nCorners_Lev[0]);
   pCorners_Lev[2] = pCorners_Lev[1] + *(nCorners_Lev[1]);
   pCorners_Lev[3] = pCorners_Lev[2] + *(nCorners_Lev[2]);

   for (int i=0; i<LEVELS; i++) {
        printf("pCorners_Lev[%d] addr: %p\n", i, pCorners_Lev[0]);   
   }
   printf("mmap()ed for Corners'Pos region\n");
*/
   // First, copy out the image data to the pyramid's zero level.
   aLevels[0].im.resize(im.size());
   copy(im, aLevels[0].im);
   
   gettimeofday(&t4, NULL);   
   // minho every frame {
   // SET the image data (im) to halfsample the image.
   // void * memset ( void * ptr, int value, size_t num );
   //memset(IMG_Lev[0], 0, 0x639c0); //SDRAM_SPAN); // memory initialize
   memcpy(IMG_Lev[0], im.begin(), im.totalsize());
   //copy(im, IMG_Lev[0]);
   //IMG_Lev[0] = (unsigned char *)im;
   //copy(aLevels[0].im, BasicImage<byte>(IMG_Lev[0], aLevels[0].im.size()));

   //printf("Copied lev.im into shared memory IMG_Lev[0]\n");
   gettimeofday(&t5, NULL);
   elapsed = timediff_msec(t4, t5);
   printf("Copying IMG_Lev[0]'duration: %f ms\n", elapsed); 

   *status_Register = 0x1; // processor write is over
   
   int status = 0;
   while(1) // polling...
   {
      status = *(status_Register);
      printf("Waiting for FPGA completed... Checking Status: %d\r", status); 

      // If status is 0, GET the halfsampled image data and corner detection result
      if (status != 1)
      {
         //printf("status : %d, frame_cnt : %d\n", status, frame_cnt);
         pCorners_Lev[1] = pCorners_Lev[0] + *(nCorners_Lev[0]);
         pCorners_Lev[2] = pCorners_Lev[1] + *(nCorners_Lev[1]);
         pCorners_Lev[3] = pCorners_Lev[2] + *(nCorners_Lev[2]);

         for (int i=0; i<LEVELS; i++) {
           printf("pCorners_Lev[%d] addr: %p\n", i, pCorners_Lev[0]);
         }
         printf("Dynamically Mapped for Corners'Pos region\n");

         // Corner detection results data 
         //int *ptr_cordet = sdram_data_ptr_cordet;
         unsigned int* ptr_cordet = nCorners_Lev[0];
         unsigned int* cornersNumber[LEVELS] = {nCorners_Lev[0], nCorners_Lev[1], nCorners_Lev[2], nCorners_Lev[3]};
         unsigned int* cornersPos[LEVELS] = {pCorners_Lev[0], pCorners_Lev[1], pCorners_Lev[2], pCorners_Lev[3]};

         // printf("Corner detection results memory : %p\n", ptr_cordet);
         //int *cordet_size[LEVELS] = { ptr_cordet, NULL, NULL, NULL };

         for (int i = 0; i < LEVELS+1; i++) // To handle SBI into Keyframe
         {
            Level &lev = aLevels[i];
	    //printf("assigning aLevels[%d] into lev..\n", i);

            if (i != 0)
            { 
               // Image data has 4 levels. It should be accessed by fixed address.
               // Image of level 0 alrady existed above code (im)
               // e.g. if base is 0000_0000,
               //      0000_0000 - 0001_2bff level 1 (320*240 = 76800 = 0001_2c00)
               //      0001_2c00 - 0001_76ff level 2 (160*120 = 19200 = 0000_4b00)
               //      0001_7700 - 0001_89bf level 3 (80*60 = 4800 = 0000_12c0)
               lev.im.resize(aLevels[i-1].im.size() / 2);       // image resize
               //copy(BasicImage<byte>(IMG_Lev[i], lev.im.size()), lev.im);  // copy image data to "lev.im"
               memcpy(lev.im.begin(), IMG_Lev[i], img_size_of_level[i]);
	       //printf("copied lev.im[%d] into IMG_Lev[%d]\n", i, i);
               //printf("Get image data of level [%d] from memory [%p]\n", i, sdram_data_ptr_level[i]);

               // corner detection result array size
               //cornersNumber[i] = cornersNumber[i-1] + *(cornersNumber[i-1]) + 1;
            }

            if (i == 4) { // Don't process since lev.4 is requird only for SBI input.
               //printf("IMG_Lev[4] copied, Break\n"); 
               break;
            }

            gettimeofday(&t0, NULL);
	    lev.vCorners.clear();
            lev.vCandidates.clear();
            lev.vMaxCorners.clear();

            // Corner detection data from memory has ONE integer(array size) and array data.
            // e.g. if base is 0001_89c0,
            //      0001_89c0 - 0001_89c3 level 0 image corner detection result array size (integer = 4 bytes)
            //      0001_89c4 - ...       corner detection result array data
            //      This leads to level 3.

            // printf("Level %d Corner detection results size : %d [%p]\n", i, *cordet_size[i], cordet_size[i]);
            // "cordet_size" means how many integers are next.
            //int *result_data = cordet_size[i] + 1;
            printf("Lev[%d]' Corner Size: %d\n", i, *nCorners_Lev[i]);
            printf("Lev[%d]' Corner Address: %p\n", i, pCorners_Lev[i]);

            unsigned int* result = cornersPos[i];// + 1;            
/*
            vector<ImageRef> levCorners;
            for (int el = 0; el < (*cornersNumber[i]) / 2; el+=2) 
            {
               // Upper 2-bytes mean X-position, Lower 2-bytes mean Y-position.     
               levCorners.push_back( ImageRef(*(result + el), *(result + el + 1)) );
               printf("Lev[%d]' Corner Address[%d]: %d, %d\n", i, el, ImageRef(*(result+el), *(result+el+1)));
            }
*/
            vector<ImageRef> levCorners;
            for (int el = 0; el < (*cornersNumber[i]); el++) {
               // Upper 2-bytes mean X-position, Lower 2-bytes mean Y-position.
               int x, y;
               x = *(result + el) >> 16;
               y = *(result + el) & 0x0000FFFF;
               levCorners.push_back(ImageRef(x, y)); 
               printf("result+el: 0x%x, x: 0x%x, y: 0x%x\n", *(result + el), x, y);	       
	    }
            
            // Assign results
            lev.vCorners = levCorners;
            // printf("[F %d][LEVEL %d] Corner detection results size : %d\n", frame_cnt, i, lev.vCorners.size());
            gettimeofday(&t1, NULL);
            elapsed = timediff_msec(t0, t1);
            printf("Asserting Corners' duration: %f ms\n", elapsed);
                        

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
           gettimeofday(&t2, NULL);

           unsigned int v=0;
           lev.vCornerRowLUT.clear();
           for(int y=0; y<lev.im.size().y; y++)
           {
              while(v < lev.vCorners.size() && y > lev.vCorners[v].y)
                 v++;
              lev.vCornerRowLUT.push_back(v);
           }
           //printf("1st stage done\n");
           gettimeofday(&t3, NULL);
           elapsed = timediff_msec(t2, t3);
           printf("Asserting RowLUT's duration: %f ms\n", elapsed); 
#if 0
        // image save temporarily { 
        if (frame_cnt == 80 || frame_cnt == 125)
        {
          // printf("Save image to png file...\n");
          string filename = boost::lexical_cast<string>(frame_cnt) + "_lev" + boost::lexical_cast<string>(i) + "_image.png";
          string save_file_path = "/home/sockit/mh/save_result/images/" + filename;
          img_save(lev.im, save_file_path);
          // cout << "Save complete...\n\tfilename: " << filename << "\n" << endl;

          // printf("Save result of corner detection of frame %ld...\n", frame_cnt);
          ofstream writeCorners;
          filename = boost::lexical_cast<string>(frame_cnt) + "_lev" + boost::lexical_cast<string>(i) + "_result_cordet.txt";
          save_file_path = "/home/sockit/mh/save_result/corner_detection_results/" + filename;
          writeCorners.open(save_file_path.c_str());
          if (writeCorners.is_open()) {
            vector<ImageRef> vc = lev.vCorners;
            for (int k = 0; k < vc.size(); k++)
            {
              char x_to_hex[5];
              char y_to_hex[5];
              sprintf(x_to_hex, "%X", vc[k].x);
              sprintf(y_to_hex, "%X", vc[k].y);
              string str;
              str.append(x_to_hex);
              str.append(",");
              str.append(y_to_hex);
              str.append("\n");
              writeCorners.write(str.c_str(), str.size());
            }
          } else
          {
            cerr << "[" << filename << "] can not open..." << endl;
          }
          cout << "Save complete...\n\tfilename: " << filename << "\n" << endl;
          writeCorners.close();  
        }
        // image save temporarily }
#endif
      }
      break;
    } // if (status != 1)

  } // while(1) 

  // } minho
}

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

// minho {
void mmap_init() 
{
   printf("mmap_init()+++\n");
   
   printf("init the variabels for mmap()...\n");
   int fd;   // "/dev/mem" file description
   off_t hps_base_offset, axi_h2f_offset;   // page offset

   // (offset % PAGE_SIZE) must equals 0.
   // In other words, the offset is a multiple of the page size.
   hps_base_offset = SDRAM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   axi_h2f_offset = AXI_LW_MEM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   printf("hps_base_offset : %p, axi_h2f_offset : %p\n", hps_base_offset, axi_h2f_offset);

   // 1. Open "/dev/mem"
   if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
   {
      printf("ERROR: could not open \"/dev/mem\"...\n");
      return;
   }
   printf("Opened \"/dev/mem\" for memmory mapped access.\n");

   // 2. mmap()
   // void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);
   hps_virtual_base = mmap(NULL, SDRAM_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, hps_base_offset);
   lw_axi_h2f_base = mmap(NULL, AXI_LW_MEM_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, axi_h2f_offset);
                             
   if (hps_virtual_base == MAP_FAILED || lw_axi_h2f_base == MAP_FAILED)
   {
      printf("ERROR: mmap() failed...\n%s\n", strerror(errno));
      close(fd);
      return;
   } 
   else
   {
      printf("hps_virtual_base : %p, lw_axi_h2f_base: %p\n", hps_virtual_base, lw_axi_h2f_base);	
   }

   //IMG_Lev[0] = (CVD::Image<CVD::byte> * )(hps_virtual_base);
   IMG_Lev[0] = (unsigned int*)hps_virtual_base;
   IMG_Lev[1] = IMG_Lev[0] + MEM_SIZE_LEV0;
   IMG_Lev[2] = IMG_Lev[1] + MEM_SIZE_LEV1;
   IMG_Lev[3] = IMG_Lev[2] + MEM_SIZE_LEV2;
   IMG_Lev[4] = IMG_Lev[3] + MEM_SIZE_LEV3;

   for (int i=0; i<LEVELS+1; i++) {
   	printf("IMG_Lev[%d] addr: %p\n", i, IMG_Lev[i]);  
   }   
   printf("mmap()ed for pyramid images\n");

   /* LW_AXI_H2F register */
   // N corners Lv.x : 32-bit unsigned integer
   status_Register = (unsigned int*)lw_axi_h2f_base;
   nCorners_Lev[0] = status_Register + N_CORNERS_LEV0_OFFSET;
   nCorners_Lev[1] = status_Register + N_CORNERS_LEV1_OFFSET;
   nCorners_Lev[2] = status_Register + N_CORNERS_LEV2_OFFSET;
   nCorners_Lev[3] = status_Register + N_CORNERS_LEV3_OFFSET;

   printf("StatusRegister addr: %p\n", status_Register);

   for (int i=0; i<LEVELS; i++) {
        printf("nCorners_Lev[%d] addr: %p\n", i, nCorners_Lev[i]);
   }
   printf("mmap()ed for AXI_H2F region\n");

   pCorners_Lev[0] = IMG_Lev[4] + MEM_SIZE_LEV4;
   printf("pCorners_Lev[0] addr: %p\n", pCorners_Lev[0]);
/*
   pCorners_Lev[1] = pCorners_Lev[0] + *(nCorners_Lev[0]);
   pCorners_Lev[2] = pCorners_Lev[1] + *(nCorners_Lev[1]);
   pCorners_Lev[3] = pCorners_Lev[2] + *(nCorners_Lev[2]);

   for (int i=0; i<LEVELS; i++) {
 	printf("pCorners_Lev[%d] addr: %p\n", i, pCorners_Lev[0]);   
   }
   printf("Mapped for Corners'Pos region\n");
*/
   /* LW_AXI_H2F register */
   //statusRegister = (unsigned char*)lw_axi_h2f_base;
   //nCorners_Lev[0] = statusRegister + N_CORNERS_LEV0_OFFSET;
   //nCorners_Lev[1] = statusRegister + N_CORNERS_LEV1_OFFSET;
   //nCorners_Lev[2] = statusRegister + N_CORNERS_LEV2_OFFSET;
   //nCorners_Lev[3] = statusRegister + N_CORNERS_LEV3_OFFSET;

   //for (int i=0; i<LEVELS; i++) {
	//rintf("nCorners_Lev[%d] addr: %p\n", i, nCorners_Lev[i]);
   //}
   //printf("Mapped for AXI_H2F region\n"); 

/*
   sdram_data_ptr_level[0] = sdram_data_ptr;
  
   for (int i = 1; i < LEVELS; i++)
   {
      sdram_data_ptr_level[i] = sdram_data_ptr_level[i-1] + img_size_of_level[i-1];
   }

   sdram_data_ptr_cordet = (int *)( sdram_data_ptr_level[3] + img_size_of_level[3] );
*/

   // Signal Handling
   memset(&sa, 0, sizeof(struct sigaction));
   sigemptyset(&sa.sa_mask);
   sa.sa_sigaction = segfault_sigaction;
   sa.sa_flags   = SA_SIGINFO;

   signal(SIGINT, on_close);         // user interrupt (Ctrl + c)
   sigaction(SIGSEGV, &sa, NULL);    // Segmentation Fault

   printf("mmap_init()---\n");
}

void on_close(int signal)
{
   my_close();
   exit(EXIT_SUCCESS);
}

void segfault_sigaction(int signal, siginfo_t *si, void *arg) 
{
   printf("Caught segfault at address %p\n", si->si_addr);
   my_close();
   exit(EXIT_SUCCESS); 
}

void my_close() 
{
   printf("\n");
   printf("================================================\n");
   printf("Ptam end...\n");

   printf("unmap the memory...\n");
   //printf("hps_virtual_base : %p\n", hps_virtual_base);
   //printf("memory size : %d\n", SDRAM_SPAN);
   munmap(hps_virtual_base, SDRAM_SPAN);
   munmap(lw_axi_h2f_base, AXI_LW_MEM_SPAN);
   printf("================================================\n");
}
// minho }





