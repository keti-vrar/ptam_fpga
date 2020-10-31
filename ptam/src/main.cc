// Copyright 2008 Isis Innovation Limited
// This is the main extry point for PTAM
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "ptam/System.h"
#include <ptam/Params.h>

#include "ros/ros.h"

// mmap related
#include <fcntl.h>   // file open option
#include <unistd.h>   // mmap
#include <sys/mman.h>   // mmap
#include <boost/lexical_cast.hpp>   // int to string
#include <signal.h>

int mmap_init();
void on_close(int signal);
void my_close();
void segfault_sigaction(int signal, siginfo_t *si, void *arg);
struct sigaction sa;

/*
void* lev0_img_map;
void* lev1_img_map;
void* lev2_img_map;
void* lev3_img_map;
void* sbi_map;
void* corners_pos_map;
void* status_reg_map;
void* lev0_corners_num_map;
void* lev1_corners_num_map;
void* lev2_corners_num_map;
void* lev3_corners_num_map;
*/
void* lev_img_map;
void* reg_map;
unsigned char* lev0_img_ptr;
unsigned char* lev1_img_ptr;
unsigned char* lev2_img_ptr;
unsigned char* lev3_img_ptr;
unsigned int* corners_pos_ptr;
unsigned int* status_reg_ptr;
unsigned int* lev0_corners_num_ptr;
unsigned int* lev1_corners_num_ptr;
unsigned int* lev2_corners_num_ptr;
unsigned int* lev3_corners_num_ptr;

int length_lev, length_reg;

using namespace std;
using namespace GVars3;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ptam");
  ROS_INFO("starting ptam with node name %s", ros::this_node::getName().c_str());

  cout << "  Welcome to PTAM " << endl;
  cout << "  --------------- " << endl;
  cout << "  Parallel tracking and mapping for Small AR workspaces" << endl;
  cout << "  Copyright (C) Isis Innovation Limited 2008 " << endl;
  cout << endl;

  GUI.StartParserThread(); // Start parsing of the console input
  atexit(GUI.StopParserThread);

  mmap_init();

  try
  {
    std::cout<<"Gui is "<<(PtamParameters::fixparams().gui ? "on" : "off")<<std::endl; //make the singleton instantiate
    System s;
    s.Run();
  }
  catch(CVD::Exceptions::All& e)
  {
    cout << endl;
    cout << "!! Failed to run system; got exception. " << endl;
    cout << "   Exception was: " << endl;
    cout << e.what << endl;
  }
}

int mmap_init() 
{
   int fd, pagesize; 
   off_t lev_offset, reg_offset;

   //off_t hps_base_offset, axi_h2f_offset;
   //off_t lev0_offset, lev1_offset, lev2_offset, lev3_offset;
   //off_t sbi_offset, corners_pos_offset, status_reg_offset;
   //off_t lev0_corners_num_offset, lev1_corners_num_offset, lev2_corners_num_offset, lev3_corners_num_offset;
   printf("mmap_init+++\n");

#if 0
   //
   // Verify that SDRAM_BASE & AXI_LW_MEM_BASE is page aligned 
   // (offset % PAGE_SIZE) must equals 0. In other words, the offset is a multiple of the page size.
   hps_base_offset = SDRAM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   axi_h2f_offset = AXI_LW_MEM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   printf("hps_base_offset : %p, axi_h2f_offset : %p\n", (void *)hps_base_offset, (void *)axi_h2f_offset);
#endif
   
   // Verify that BASEs are page aligned
#if 0 
   lev0_offset = (LEV0_IMG_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1));
   lev1_offset = (LEV1_IMG_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1));
   lev2_offset = (LEV2_IMG_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1));
   lev3_offset = (LEV3_IMG_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1));
   sbi_offset = (SBI_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1));
   corners_pos_offset = (CORNERS_POS_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1));
   status_reg_offset = (STATUS_REG_BASE * ~(sysconf(_SC_PAGE_SIZE) - 1));
   lev0_corners_num_offset = (LEV0_CORNERS_NUM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1));
   lev1_corners_num_offset = (LEV1_CORNERS_NUM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1));
   lev2_corners_num_offset = (LEV2_CORNERS_NUM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1));
   lev3_corners_num_offset = (LEV3_CORNERS_NUM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1));
#else 
   pagesize = sysconf(_SC_PAGESIZE);
   lev_offset = LEV_IMG_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   reg_offset = REG_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1); 
#endif
 
   // open() the /dev/mem device
   if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
      printf("Device Open Failed\n");
      return (1);
   } else {
      printf("Device Open Success\n");
   }

#if 0   
   //buffer = calloc(AXI_LW_MEM_SPAN, sizeof(*buffer));
   //
   // mmap() the base of lw_axi_h2f_base hardware
   // 
   //lw_axi_h2f_base = (void *)malloc(AXI_LW_MEM_SPAN);
   lw_axi_h2f_base = mmap64(NULL, AXI_LW_MEM_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, axi_h2f_offset);

   // mmap() the base of hps_virtual_base hardware
   // void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);
   //
   //hps_virtual_base = (void *)malloc(4096 * 4000);
   hps_virtual_base = mmap64(NULL, 
			(4096 * 4000),  //SDRAM_SPAN, 
			(PROT_READ | PROT_WRITE), 
			MAP_SHARED, 
			fd, 
			0x0);//hps_base_offset);
                             
   if (hps_virtual_base == MAP_FAILED || lw_axi_h2f_base == MAP_FAILED) {
      printf("Mapping Failed\n");
      close(fd);
      return (1);
   } else {
      printf("hps_virtual_base : %p, lw_axi_h2f_base: %p\n", (void *)hps_virtual_base, (void *)lw_axi_h2f_base);	
   }
#endif

   // memory mapping hardware
   //off_t lev0_offset, lev1_offset, lev2_offset, lev3_offseta;
   //off_t sbi_offset, corners_pos_offset, status_reg_offset;
   //off_t lev0_corners_num_offset, lev1_corners_num_offset, lev2_corners_num_offset, lev3_corners_num_offset;
#if 0
   lev0_img_map = mmap(NULL, LEV0_IMG_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, lev0_offset);
   if (lev0_img_map == MAP_FAILED) {
      printf("lev0_img_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   lev1_img_map = mmap(NULL, LEV1_IMG_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, lev1_offset);
   if (lev1_img_map == MAP_FAILED) {
      printf("lev1_img_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   lev2_img_map = mmap(NULL, LEV2_IMG_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, lev2_offset);
   if (lev2_img_map == MAP_FAILED) {
      printf("lev2_img_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   lev3_img_map = mmap(NULL, LEV3_IMG_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, lev3_offset);
   if (lev3_img_map == MAP_FAILED) {
      printf("lev3_img_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   sbi_map = mmap(NULL, SBI_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, sbi_offset);
   if (sbi_map == MAP_FAILED) {
      printf("sbi_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   corners_pos_map = mmap(NULL, CORNERS_POS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, corners_pos_offset);
   if (corners_pos_map == MAP_FAILED) {
      printf("corners_pos_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   status_reg_map = mmap(NULL, 4096/*STATUS_REG_SPAN*/, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, status_reg_offset);
   if (status_reg_map == MAP_FAILED) {
      printf("status_reg_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   lev0_corners_num_map = mmap(NULL, 4096/*LEV0_CORNERS_NUM_SPAN*/, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, lev0_corners_num_offset);
   if (lev0_corners_num_map == MAP_FAILED) {
      printf("lev0_corners_num_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   lev1_corners_num_map = mmap(NULL, 4096/*LEV1_CORNERS_NUM_SPAN*/, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, lev1_corners_num_offset);
   if (lev1_corners_num_map == MAP_FAILED) {
      printf("lev1_corners_num_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   lev2_corners_num_map = mmap(NULL, 4096/*LEV2_CORNERS_NUM_SPAN*/, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, lev2_corners_num_offset);
   if (lev2_corners_num_map == MAP_FAILED) {
      printf("lev2_corners_num_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   lev3_corners_num_map = mmap(NULL, 4096/*LEV3_CORNERS_NUM_SPAN*/, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, lev3_corners_num_offset);
   if (lev3_corners_num_map == MAP_FAILED) {
      printf("lev3_corners_num_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }

   printf("&lev0_img_map: %p\n", &lev0_img_map);
   printf("lev1_img_map: %p\n", &lev1_img_map);
   printf("lev2_img_map: %p\n", &lev2_img_map);
   printf("lev3_img_map: %p\n", &lev3_img_map);
   printf("sbi_map: %p\n", &sbi_map);
   printf("corners_pos_map: %p\n", &corners_pos_map);
   printf("status_reg_map: %p\n", &status_reg_map);
   printf("lev0_corners_num_map: %p\n", &lev0_corners_num_map);
   printf("lev1_corners_num_map: %p\n", &lev1_corners_num_map);
   printf("lev2_corners_num_map: %p\n", &lev2_corners_num_map);
   printf("lev3_corners_num_map: %p\n", &lev3_corners_num_map);
#else
   length_lev = pagesize * 100;
   lev_img_map = mmap(NULL, length_lev, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, lev_offset);
   if (lev_img_map == MAP_FAILED) {
      printf("lev0_img_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   
   length_reg = pagesize * 1;
   reg_map = mmap(NULL, length_reg, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, reg_offset);
   if (reg_map == MAP_FAILED) {
      printf("lev1_img_map mapping failed\n");
      close(fd);
      exit(EXIT_FAILURE);
   }
   printf("lev_img_map: %p\n", (void *)lev_img_map);
   printf("reg_map: %p\n", (void *)reg_map);

   lev0_img_ptr = static_cast<unsigned char *>(lev_img_map);
   lev1_img_ptr = static_cast<unsigned char *>(lev_img_map + LEV1_OFFSET);
   lev2_img_ptr = static_cast<unsigned char *>(lev_img_map + LEV2_OFFSET);
   lev3_img_ptr = static_cast<unsigned char *>(lev_img_map + LEV3_OFFSET);
   corners_pos_ptr = static_cast<unsigned int *>(lev_img_map + CORNERS_POS_BASE);
   status_reg_ptr = static_cast<unsigned int *>(reg_map);
   lev0_corners_num_ptr = static_cast<unsigned int *>(reg_map + LEV0_CORNERS_NUM_OFFSET);
   lev1_corners_num_ptr = static_cast<unsigned int *>(reg_map + LEV1_CORNERS_NUM_OFFSET);
   lev2_corners_num_ptr = static_cast<unsigned int *>(reg_map + LEV2_CORNERS_NUM_OFFSET);
   lev3_corners_num_ptr = static_cast<unsigned int *>(reg_map + LEV3_CORNERS_NUM_OFFSET);

   printf("lev0_img_ptr: %p\n", lev0_img_ptr);
   printf("lev1_img_ptr: %p\n", lev1_img_ptr);
   printf("lev2_img_ptr: %p\n", lev2_img_ptr);
   printf("lev3_img_ptr: %p\n", lev3_img_ptr);
   printf("corners_pos_ptr: %p\n", corners_pos_ptr);
   printf("status_reg_ptr: %p\n", status_reg_ptr);
   printf("n_corners_lv0_ptr: %p\n", lev0_corners_num_ptr);
   printf("n_corners_lv1_ptr: %p\n", lev1_corners_num_ptr);
   printf("n_corners_lv2_ptr: %p\n", lev2_corners_num_ptr);
   printf("n_corners_lv3_ptr: %p\n", lev3_corners_num_ptr);
#endif

#if 0
   //
   // Use the mmap()'ed pointer like any other pointer
   // Assign Image' virtual base address
   // lev0_image_base = (volatile CVD::Image<CVD::byte>*)(hps_virtual_base);
   //
   if (hps_virtual_base != NULL) {
      lev0_img_ptr = static_cast<unsigned char *>(hps_virtual_base);
      lev1_img_ptr = static_cast<unsigned char *>(hps_virtual_base + LEV1_OFFSET);
      lev2_img_ptr = static_cast<unsigned char *>(hps_virtual_base + LEV2_OFFSET);
      lev3_img_ptr = static_cast<unsigned char *>(hps_virtual_base + LEV3_OFFSET);
      //lev4_img_ptr = (ucharptr)(hps_virtual_base + LEV4_OFFSET);
  
      // Assign corner data address
      corners_pos_ptr = static_cast<unsigned int *>(hps_virtual_base + CORNERS_POS_OFFSET);
   }

   if (lw_axi_h2f_base != NULL) {
      /* LW_AXI_H2F register */
      // N corners Lv.x : 32-bit unsigned integer
      status_register_ptr = static_cast<unsigned int *>(lw_axi_h2f_base + STATUS_REG_OFFSET);

      lev0_corners_num_ptr = static_cast<unsigned int *>(lw_axi_h2f_base + LEV0_CORNERS_NUM_OFFSET);
      lev1_corners_num_ptr = static_cast<unsigned int *>(lw_axi_h2f_base + LEV1_CORNERS_NUM_OFFSET);
      lev2_corners_num_ptr = static_cast<unsigned int *>(lw_axi_h2f_base + LEV2_CORNERS_NUM_OFFSET);
      lev3_corners_num_ptr = static_cast<unsigned int *>(lw_axi_h2f_base + LEV3_CORNERS_NUM_OFFSET);
   }

   printf("lev0_img_ptr: %p\n", (void *)lev0_img_ptr);
   printf("lev1_img_ptr: %p\n", (void *)lev1_img_ptr);
   printf("lev2_img_ptr: %p\n", (void *)lev2_img_ptr);
   printf("lev3_img_ptr: %p\n", (void *)lev3_img_ptr);
   //printf("lev4_img_ptr: %p\n", lev4_img_ptr);
   printf("corners_pos_ptr: %p\n", (void *)corners_pos_ptr);
   printf("status_register_ptr: %p\n", (void *)status_register_ptr);
   printf("n_corners_lv0_ptr: %p\n", (void *)lev0_corners_num_ptr);
   printf("n_corners_lv1_ptr: %p\n", (void *)lev1_corners_num_ptr);
   printf("n_corners_lv2_ptr: %p\n", (void *)lev2_corners_num_ptr);
   printf("n_corners_lv3_ptr: %p\n", (void *)lev3_corners_num_ptr);
#endif

   // Signal Handling
   memset(&sa, 0, sizeof(struct sigaction));
   sigemptyset(&sa.sa_mask);
   sa.sa_sigaction = segfault_sigaction;
   sa.sa_flags   = SA_RESTART | SA_SIGINFO;
   signal(SIGINT, on_close);         // user interrupt (Ctrl + c)
   sigaction(SIGSEGV, &sa, NULL);    // segmentation fault

   if (sigaction(SIGSEGV, &sa, (struct sigaction *) NULL) != 0) {
      fprintf(stderr, "error setting signal handler for %d (%s)\n", SIGSEGV, strsignal(SIGSEGV));
      exit(EXIT_FAILURE);
   }

   printf("mmap_init---\n");
   
   return 0;
}

void on_close(int signal) {
   my_close();
   exit(EXIT_SUCCESS);
}
 
void segfault_sigaction(int signal, siginfo_t *si, void *arg) {
   printf("caught Segfault at address %p\n", si->si_addr);
   my_close();
   exit(EXIT_SUCCESS); 
}
 
void my_close() {
   printf("\n");
   printf("================================================\n");
   printf("PTAM finished\n");

   printf("Unmap the memory...\n");
   //printf("hps_virtual_base : %p\n", hps_virtual_base);
   //printf("memory size : %d\n", SDRAM_SPAN);
   //*(status_register_ptr) = 0x0;
 
   //munmap(hps_virtual_base, SDRAM_SPAN);
   //munmap(lw_axi_h2f_base, AXI_LW_MEM_SPAN);
#if 0
   munmap(lev0_img_map, LEV0_IMG_SPAN);
   munmap(lev1_img_map, LEV1_IMG_SPAN);
   munmap(lev2_img_map, LEV2_IMG_SPAN);
   munmap(lev3_img_map, LEV3_IMG_SPAN);
   munmap(sbi_map, SBI_SPAN);
   munmap(corners_pos_map, CORNERS_POS_SPAN);
   munmap(status_reg_map, STATUS_REG_SPAN);
   munmap(lev0_corners_num_map, LEV0_CORNERS_NUM_SPAN);
   munmap(lev1_corners_num_map, LEV1_CORNERS_NUM_SPAN);
   munmap(lev2_corners_num_map, LEV2_CORNERS_NUM_SPAN);
   munmap(lev3_corners_num_map, LEV3_CORNERS_NUM_SPAN);
#else 
   munmap(lev_img_map, length_lev);
   munmap(reg_map, length_reg);
#endif   
   printf("================================================\n");
}









