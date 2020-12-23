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

#include <cvd/image_io.h>

using namespace std;
using namespace GVars3;
using namespace CVD;

int mmap_init();
void on_close(int signal);
void my_close();
void segfault_sigaction(int signal, siginfo_t *si, void *arg);
struct sigaction sa;

//void* lev_img_map;
//void* reg_map;
void* h2f_virtual_base;
void* f2h_virtual_base;
void* lwh2f_virtual_base;

unsigned char* lev0_img_ptr;
unsigned char* lev1_img_ptr;
unsigned char* lev2_img_ptr;
unsigned char* lev3_img_ptr;
unsigned char* lev4_img_ptr;

unsigned int* corners_pos_ptr;
unsigned int* status_reg_ptr;
unsigned int* lev0_corners_num_ptr;
unsigned int* lev1_corners_num_ptr;
unsigned int* lev2_corners_num_ptr;
unsigned int* lev3_corners_num_ptr;

int length_lev, length_reg;


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
   //off_t lev_offset, reg_offset;
   off_t f2h_offset, h2f_offset, lwh2f_offset;

   printf("mmap_init+++\n");

#if 0 // original code
   //
   // Verify that SDRAM_BASE & AXI_LW_MEM_BASE is page aligned 
   // (offset % PAGE_SIZE) must equals 0. In other words, the offset is a multiple of the page size.
   hps_base_offset = SDRAM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   axi_h2f_offset = AXI_LW_MEM_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   printf("hps_base_offset : %p, axi_h2f_offset : %p\n", (void *)hps_base_offset, (void *)axi_h2f_offset);
#endif
   
   //
   // Verify that BASEs are page aligned
   //
   pagesize = sysconf(_SC_PAGESIZE);
   //lev_offset = LEV_IMG_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   //reg_offset = REG_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1); 
   f2h_offset = F2H_BRIDGE_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   h2f_offset = H2F_BRIDGE_BASE & ~(sysconf(_SC_PAGE_SIZE) - 1);
   lwh2f_offset = LWH2F_BRIDGE_BASE & ~(sysconf(_SC_PAGE_SIZE) -1);

   //
   // open() the /dev/mem device
   //
   if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
      cout << "/dev/mem open failed" << endl;
      return (1);
   } else {
      cout << "/dev/mem open success" << endl;
   }

   //
   // memory mapping for hardware 
   //
#if 0
   length_lev = pagesize * 150;
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
#endif
   f2h_virtual_base = mmap(NULL, F2H_BRIDGE_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, f2h_offset);
   h2f_virtual_base = mmap(NULL, H2F_BRIDGE_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, h2f_offset);
   lwh2f_virtual_base = mmap(NULL, LWH2F_BRIDGE_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, lwh2f_offset); 
  
   if (f2h_virtual_base == MAP_FAILED) { //|| ocm_virtual_base || f2h_v    irtual_base || axi_lw_virtual_base) {
      cout << "ERROR: mmap() failed on f2h_virtual" << endl;
      close(fd);
      return (1);
   } else if (h2f_virtual_base == MAP_FAILED) {
      cout << "ERROR: mmap() failed on h2f_virtual" << endl;
      close(fd);
      return (1);
   } else if (lwh2f_virtual_base == MAP_FAILED) {
      cout << "ERROR: mmap() failed on lwh2f_virtual" << endl;
      close(fd);
      return (1);
   } else {
      printf("f2h_virtual_base : 0x%p\n", f2h_virtual_base);
      printf("h2f_virtual_base : 0x%p\n", h2f_virtual_base);
      printf("lwh2f_virtual_base : 0x%p\n", lwh2f_virtual_base);
   }

   //
   // Use the mmap()'ed pointer like any other pointer
   // Assign Image' virtual base address
   //
#if 0
   if (lev_img_map != NULL) {
      lev0_img_ptr = static_cast<unsigned char*>(lev_img_map);
      lev1_img_ptr = static_cast<unsigned char*>(lev_img_map + LEV1_OFFSET);
      lev2_img_ptr = static_cast<unsigned char*>(lev_img_map + LEV2_OFFSET);
      lev3_img_ptr = static_cast<unsigned char*>(lev_img_map + LEV3_OFFSET);
      corners_pos_ptr = static_cast<unsigned int*>(lev_img_map + CORNERS_POS_BASE);
   } else {
      close(fd);
      exit(EXIT_FAILURE);
   }
   
   if (reg_map != NULL) {
      status_reg_ptr = static_cast<unsigned int*>(reg_map);
      lev0_corners_num_ptr = static_cast<unsigned int*>(reg_map + LEV0_CORNERS_NUM_OFFSET);
      lev1_corners_num_ptr = static_cast<unsigned int*>(reg_map + LEV1_CORNERS_NUM_OFFSET);
      lev2_corners_num_ptr = static_cast<unsigned int*>(reg_map + LEV2_CORNERS_NUM_OFFSET);
      lev3_corners_num_ptr = static_cast<unsigned int*>(reg_map + LEV3_CORNERS_NUM_OFFSET);
   } else {
      close(fd);
      exit(EXIT_FAILURE);
   }
#endif
   if (f2h_virtual_base != NULL) {
      lev0_img_ptr = static_cast<unsigned char*>(f2h_virtual_base);
   } else {
      close(fd);
      exit(EXIT_FAILURE);
   }
   // lev1~4 images via h2f_virtual_base
   if (h2f_virtual_base != NULL) {
      lev1_img_ptr = static_cast<unsigned char*>(h2f_virtual_base);             // 0x9000_0000
      lev2_img_ptr = static_cast<unsigned char*>(h2f_virtual_base + 0x12C00);   // 0x9001_2C00
      lev3_img_ptr = static_cast<unsigned char*>(h2f_virtual_base + 0x17700);   // 0x9001_7700
      lev4_img_ptr = static_cast<unsigned char*>(h2f_virtual_base + 0x189C0);   // 0x9001_89C0

      lev0_corners_num_ptr = static_cast<unsigned int*>(h2f_virtual_base + 0x18E70);  // 0xF900_0004
      lev1_corners_num_ptr = static_cast<unsigned int*>(h2f_virtual_base + 0x18E74);  // 0xF900_0008
      lev2_corners_num_ptr = static_cast<unsigned int*>(h2f_virtual_base + 0x18E78);  // 0xF900_000C
      lev3_corners_num_ptr = static_cast<unsigned int*>(h2f_virtual_base + 0x18E7C); // 0xF900_0010

      corners_pos_ptr = static_cast<unsigned int*>(h2f_virtual_base + 0x18E80); // 0x9001_8E70
   } else {
      close(fd);
      exit(EXIT_FAILURE);
   }
   // 
   if (lwh2f_virtual_base != NULL) {
      status_reg_ptr = static_cast<unsigned int*>(lwh2f_virtual_base);              // 0xF900_0000
      //lev0_corners_num_ptr = static_cast<unsigned int*>(lwh2f_virtual_base + 0x4);  // 0xF900_0004
      //lev1_corners_num_ptr = static_cast<unsigned int*>(lwh2f_virtual_base + 0x8);  // 0xF900_0008
      //lev2_corners_num_ptr = static_cast<unsigned int*>(lwh2f_virtual_base + 0xC);  // 0xF900_000C
      //lev3_corners_num_ptr = static_cast<unsigned int*>(lwh2f_virtual_base + 0x10); // 0xF900_0010
   } else {
      close(fd);
      exit(EXIT_FAILURE);
   }

   // Check the assigned virtual addres
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

   // sleep after FPGA reset
   *(status_reg_ptr) = 0x80000000;
   usleep(1000);
   *(status_reg_ptr) = 0x00000000;

   // Signal Handling
   memset(&sa, 0, sizeof(struct sigaction));
   sigemptyset(&sa.sa_mask);
   sa.sa_sigaction = segfault_sigaction;
   sa.sa_flags   = SA_RESTART | SA_SIGINFO;
   signal(SIGINT, on_close);         // user interrupt (Ctrl + c)
   sigaction(SIGSEGV, &sa, NULL);    // segmentation fault
/*
   if (sigaction(SIGSEGV, &sa, (struct sigaction *) NULL) != 0) {
      fprintf(stderr, "error setting signal handler for %d (%s)\n", SIGSEGV, strsignal(SIGSEGV));
      exit(EXIT_FAILURE);
   }
*/
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

   //*(status_reg_ptr) = 0x80000000;
   //usleep(1000);
   //*(status_reg_ptr) = 0x00000000;

   //printf("hps_virtual_base : %p\n", hps_virtual_base);
   //printf("memory size : %d\n", SDRAM_SPAN);
   //*(status_register_ptr) = 0x0;
 
   //munmap(hps_virtual_base, SDRAM_SPAN);
   //munmap(lw_axi_h2f_base, AXI_LW_MEM_SPAN);
   //munmap(lev_img_map, length_lev);
   //munmap(reg_map, length_reg);
   munmap(f2h_virtual_base, F2H_BRIDGE_SPAN);
   munmap(h2f_virtual_base, H2F_BRIDGE_SPAN);
   munmap(lwh2f_virtual_base, LWH2F_BRIDGE_SPAN);
   printf("================================================\n");
}









