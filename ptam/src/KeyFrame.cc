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
// }

using namespace CVD;
using namespace std;

//#define IMG_SIZE_640X480 0x0004b000     // 307200 bytes
//#define IMG_SIZE_320X240 0x00012C00     // 76800 bytes
//#define IMG_SIZE_160X120 0x00004B00     // 19200 bytes
//#define IMG_SIZE_80X60 0x000012C0       // 4800 bytes   // total 408000 (0006_39C0)

static bool is_first_frame = true;
static unsigned long int frame_cnt = 0;

struct sigaction sa;

void mmap_init();
void on_close(int signal);
void my_close();
void segfault_sigaction(int signal, siginfo_t *si, void *arg);

void* hps_virtual_base;
void* lw_axi_h2f_base;
//volatile CVD::Image<CVD::byte>* lev0_image_base;
unsigned char * lev0_image_base = 0;
unsigned char * lev1_image_base = 0;
unsigned char * lev2_image_base = 0;
unsigned char * lev3_image_base = 0;
unsigned char * lev4_image_base = 0;
unsigned int * addr_corners_base = 0;
unsigned int * status_register = 0;
unsigned int * n_corners_lv0 = 0;
unsigned int * n_corners_lv1 = 0;
unsigned int * n_corners_lv2 = 0;
unsigned int * n_corners_lv3 = 0;

unsigned int* IMG_Lev[LEVELS+1] = {0,};
//CVD::Image<CVD::byte> *IMG_Lev[LEVELS+1] = {};

unsigned int* pCorners_Lev[LEVELS] = {0,};
unsigned int* nCorners_Lev[LEVELS] = {0,};

unsigned int* status_Register = 0;	
unsigned int* cornerNumber = 0;


void mmap_init() {  
   int fd;   // "/dev/mem" file description
   off_t hps_base_offset, axi_h2f_offset; // page offset
   cout << "mmap_init()+++" << endl;
   //cout << "init the variabels for mmap()..." << endl;

   //
   // Verify that SDRAM_BASE & AXI_LW_MEM_BASE is page aligned 
   //
   // (offset % PAGE_SIZE) must equals 0.
   // In other words, the offset is a multiple of the page size.
   hps_base_offset = SDRAM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   axi_h2f_offset = AXI_LW_MEM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   printf("hps_base_offset : %p, axi_h2f_offset : %p\n", (void *)hps_base_offset, axi_h2f_offset);

   //
   // open() the /dev/mem device
   //
   if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
      //printf("ERROR: could not open \"/dev/mem\"...\n");
      cout << "devmem open error" << endl;
      exit(EXIT_FAILURE);
      //return;
   }
   printf("Opened \"/dev/mem\" for memmory mapped access.\n");

   //
   // mmap() the base of hps_virtual_base hardware
   // 
   // void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);
   hps_virtual_base = mmap(NULL, 
			SDRAM_SPAN, 
			(PROT_READ | PROT_WRITE), 
			MAP_SHARED, 
			fd, 
			hps_base_offset);
   //
   // mmap() the base of lw_axi_h2f_base hardware
   // 
   lw_axi_h2f_base = mmap(NULL, 
			AXI_LW_MEM_SPAN,
			(PROT_READ | PROT_WRITE),
			MAP_SHARED, 
			fd, 
			axi_h2f_offset);
                             
   if (hps_virtual_base == MAP_FAILED || lw_axi_h2f_base == MAP_FAILED) {
      //printf("ERROR: mmap() failed...\n%s\n", strerror(errno));
      cout << "devmem mmap error" << endl;
      close(fd);
      exit(EXIT_FAILURE);
      //return;
   } else {
      printf("hps_virtual_base : %p, lw_axi_h2f_base: %p\n", hps_virtual_base, lw_axi_h2f_base);	
   }

   // 
   // Use the mmap()'ed pointer like any other pointer
   //
   // Assign Image' virtual base address
   // lev0_image_base = (volatile CVD::Image<CVD::byte>*)(hps_virtual_base);
   lev0_image_base = (unsigned char *)(hps_virtual_base);
   lev1_image_base = (unsigned char *)(hps_virtual_base + LEV1_IMG_BASE);
   lev2_image_base = (unsigned char *)(hps_virtual_base + LEV2_IMG_BASE);
   lev3_image_base = (unsigned char *)(hps_virtual_base + LEV3_IMG_BASE);
   lev4_image_base = (unsigned char *)(hps_virtual_base + LEV4_IMG_BASE);

   // Assign corner data address
   addr_corners_base = (unsigned int *) (hps_virtual_base + ADDR_CORNERS_BASE);
   
   /* LW_AXI_H2F register */
   // N corners Lv.x : 32-bit unsigned integer
   status_register = (unsigned int *)(lw_axi_h2f_base);
   n_corners_lv0 = (unsigned int *)(lw_axi_h2f_base + N_CORNERS_LEV0_OFST);
   n_corners_lv1 = (unsigned int *)(lw_axi_h2f_base + N_CORNERS_LEV1_OFST);
   n_corners_lv2 = (unsigned int *)(lw_axi_h2f_base + N_CORNERS_LEV2_OFST);
   n_corners_lv3 = (unsigned int *)(lw_axi_h2f_base + N_CORNERS_LEV3_OFST);

   printf("lev0_image_base: %p\n", lev0_image_base);
   printf("lev1_image_base: %p\n", lev1_image_base);
   printf("lev2_image_base: %p\n", lev2_image_base);
   printf("lev3_image_base: %p\n", lev3_image_base);
   printf("lev4_image_base: %p\n", lev4_image_base);
   printf("status_register addr: %p\n", status_register);
   
#if 0
   sdram_data_ptr_level[0] = sdram_data_ptr;
  
   for (int i = 1; i < LEVELS; i++)
   {
      sdram_data_ptr_level[i] = sdram_data_ptr_level[i-1] + img_size_of_level[i-1];
   }

   sdram_data_ptr_cordet = (int *)( sdram_data_ptr_level[3] + img_size_of_level[3] );
#endif

   // Signal Handling
   memset(&sa, 0, sizeof(struct sigaction));
   sigemptyset(&sa.sa_mask);
   sa.sa_sigaction = segfault_sigaction;
   sa.sa_flags   = SA_SIGINFO;

   signal(SIGINT, on_close);         // user interrupt (Ctrl + c)
   sigaction(SIGSEGV, &sa, NULL);    // Segmentation Fault

   printf("mmap_init()---\n");
}

void on_close(int signal) {
   my_close();
   exit(EXIT_SUCCESS);
}

void segfault_sigaction(int signal, siginfo_t *si, void *arg) {
   printf("Caught segfault at address %p\n", si->si_addr);
   my_close();
   exit(EXIT_SUCCESS); 
}

void my_close() {
   printf("\n");
   printf("================================================\n");
   printf("Ptam end...\n");

   printf("unmap the memory...\n");
   //printf("hps_virtual_base : %p\n", hps_virtual_base);
   //printf("memory size : %d\n", SDRAM_SPAN);
   *(status_register) = 0x0;

   munmap(hps_virtual_base, SDRAM_SPAN);
   munmap(lw_axi_h2f_base, AXI_LW_MEM_SPAN);
   printf("================================================\n");
}

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
   float elapsed;
   ImageRef pos;

   //unsigned char * levels_image[5] = {0,};
   //unsigned int * number_corners[4] = {0,};    
   //unsigned int * addr_corners[4] = {0,};

   frame_cnt++;
   cout << "frame_cnt: " << frame_cnt << endl;

   if (is_first_frame) {
      mmap_init();
      is_first_frame = false;
   }
   
   // First, copy out the image data to the pyramid's zero level.
   cout << "im.resize()" << endl;
   aLevels[0].im.resize(im.size());

   cout << "copy im to aLevels[0].im" << endl;
   copy(im, aLevels[0].im);
#if 1
   if (lev0_image_base != NULL) {
      memcpy(lev0_image_base, im.begin(), im.totalsize());
   }
   //lev0_image_base = &im;
#else   
   //cout << "copy lev[0] img to mmap()'ed region" << endl;
   for (int y=0; y<im.size().y; y++) {
      for (int x=0; x<im.size().x; x++) {
         pos.x = x; pos.y = y;
         *(lev0_image_base + (y*im.size().x+x)) = im[pos]; // assert unsigned char into volatile pointer
      }
   }
#endif

#if 0
   //printf("pix[0] addr: %p\n", &(lev0_image_base[0]));
   //printf("pix[1] addr: %p\n", &(lev0_image_base[1]));
   //printf("pix[640*480] addr: %p\n", &(lev0_image_base[640*480-1]));

   //copy(im, lev0_image_base);

   for (int y=0; y<im.size().y; y++) {
      for (int x=0; x<im.size().x; x++) {
         //pos.x = x; pos.y = y;
         *(lev0_image_base + (y*im.size().x+x)) = 0;
         bufb[y*im.size().x+x] = *(lev0_image_base + (y*im.size().x+x)); 
      }
   }

   int comp_result = 0;
   comp_result = (bufa[0] == bufb[0] && bufa[307200-1] == bufb[307200-1]);
   printf("comp_result: %d, (1:same, 0:diff)", comp_result);

   //gettimeofday(&t5, NULL);
   //elapsed = timediff_msec(t4, t5);
   //printf("Copyed im to FPGA: %f ms\n", elapsed); 
#endif

   cout << "Set bit" << endl;
   *(status_register) = 0x1; // processor write is over

   while (true) {
      if (*(status_register) == 0x3) {
  	cout << "status: ok" << endl; 
	break;
      }
      printf("waiting: %d\r", *(status_register));
      usleep(5000);
   }
   
   usleep(5000);
   cout << "Clear bit" << endl;
   *(status_register) = 0x0; // Clear status_register
   usleep(5000);

   cout << "copy the processed data back" << endl;

   for (int i = 0; i < LEVELS; i++) // To handle SBI into Keyframe
   {
      //cout << "begin of lev[" << i << "]" <<  endl;
      Level &lev = aLevels[i];
      
      if (i != 0) {
         lev.im.resize(aLevels[i-1].im.size() / 2);       // image resize
#if 1
         for (int y=0; y< lev.im.size().y; y++) {
            for (int x=0; x< lev.im.size().x; x++) {
               pos.x = x; pos.y = y;
               if ( i == 1) {
                  lev.im[pos] = *(lev1_image_base + (y*lev.im.size().x+x));
               } else if ( i == 2) {
                  lev.im[pos] = *(lev2_image_base + (y*lev.im.size().x+x));
               } else if ( i == 3) {
                  lev.im[pos] = *(lev3_image_base + (y*lev.im.size().x+x));
               } 
//               else if ( i == 4) {
//                lev.im[pos] = *(lev4_image_base + (y*lev.im.size().x+x));
//             }
            }
         }
#else
	copy(BasicImage<byte>(levels_image[i], lev.im.size()), lev.im);
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
#if 0       
      cout << "Update corner number & address" << endl;
      if (i == 0) {
         number_corners[0] = n_corners_lv0;
         addr_corners[0] = addr_corners_base;
      } else if (i == 1) {
         number_corners[1] = n_corners_lv1;
         addr_corners[1] = addr_corners_base + *(n_corners_lv0) + 1;
      } else if (i == 2) {
         number_corners[2] = n_corners_lv2;
         addr_corners[2] = addr_corners_base + *(n_corners_lv0) + *(n_corners_lv1) + 1;
      } else if (i == 3) {
         number_corners[3] = n_corners_lv3;
         addr_corners[3] = addr_corners_base + *(n_corners_lv0) + *(n_corners_lv1) + *(n_corners_lv2) + 1;
      } else {
         break;
      }
#endif

#if 0
      printf("addr_corners_base: %p\n", addr_corners_base);
      printf("n_corners_lev0 addr: %p\n", n_corners_lv0);
      printf("n_corners_lev1 addr: %p\n", n_corners_lv1);
      printf("n_corners_lev2 addr: %p\n", n_corners_lv2);
      printf("n_corners_lev3 addr: %p\n", n_corners_lv3); 
#endif
      volatile unsigned int * result; // = addr_corners[i];
      volatile unsigned int num; // = *(number_corners[i]); 
      
      if (i == 0) {
         result = addr_corners_base;
         num = *(n_corners_lv0);
      } else if (i == 1) {
         result = addr_corners_base + *(n_corners_lv0) + 1; 
         num = *(n_corners_lv1);
      } else if (i == 2) {
         result = addr_corners_base + *(n_corners_lv0) + *(n_corners_lv1) + 1;
         num = *(n_corners_lv2);
      } else if (i == 3) {
         result = addr_corners_base + *(n_corners_lv0) + *(n_corners_lv1) + *(n_corners_lv2) + 1;
         num = *(n_corners_lv3);
      } else {
        
      }

      vector<ImageRef> levCorners;
      levCorners.resize(num); // testing for heap corruption

      for (int el = 0; el < num; el++) {
         // Upper 2-bytes mean X-position, Lower 2-bytes mean Y-position.
         ImageRef corner_pos;
         corner_pos.x = *(result + el) >> 16;
         corner_pos.y = *(result + el) & 0x0000FFFF;
         levCorners.push_back(corner_pos);
         //printf("result+el: 0x%x, x: 0x%x, y: 0x%x\n", *(result + el), corner_pos.x, corner_pos.y);
      }
      //printf("number_corners[%d]'size: %d\n", i, *(number_corners[i]));

      // Assign results
      lev.vCorners = levCorners;
      cout << "lev[" << i << "] " << lev.vCorners.size() << endl;

      //cout << lev.vCorners.at(0) << lev.vCorners.at(1) << lev.vCorners.at(2) << lev.vCorners.at(3) << endl;

      //printf("[F %d][LEVEL %d] Corner detection results size : %d\n", frame_cnt, i, lev.vCorners.size());
      //gettimeofday(&t1, NULL);
      //elapsed = timediff_msec(t0, t1);
      //printf("Asserting Corners' duration: %f ms\n", elapsed);

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
     
      //cout << "end of lev[" << i << "] " << endl;
#endif
      //printf("1st stage done\n");
      //gettimeofday(&t3, NULL);
      //elapsed = timediff_msec(t2, t3);
      //printf("Asserting RowLUT's duration: %f ms\n", elapsed); 

   } //end of for-loop
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





