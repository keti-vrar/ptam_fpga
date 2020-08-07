ptam_fpgasoc
============
This repository is applicable to ubuntu core 16.04 based boards. 
The confirmed H/W references are the Intel Stratix 10 and Nanopc-T4 RK3399 Lubuntu 16.04.x installed.

For RK3399 Lubuntu environment need to set the following instruction to run usb_cam.
"#rosrun usb_cam usb_cam_node _video_device:=/dev/video10"

[How to connect Master] 
1. #ssh sockit@192.168.1.111 (from Remote Host)
2. #su 

[How to Install] 
1. #mkdir -p your_workspace/catkin_ws/src
2. #cd your_workspace/catkin_ws/src
3. #git clone https://github.com/keti-vrar/ptam_fpgasoc
4. #cd ..
5. #catkin_make

[How to Run]
1. #source devel/setup.sh
2. #roscore  (from Stratix10)
3. #roscore  (from Host. e.g. Ubutu16 based remote machine)
4. #roslaunch ptam ptam.launch // (from Stratix10 using another termial)
5. #rosrun ptam remote_ptam (from Remote Host)
6. #rosrun usb_cam usb_cam_node (from Remote Host)
7. #rosrun ROS_NAMESPACE=usb_cam rosrun image_proc image_proc (from Remote Host)




[Updated on Jul/15th]

The baseline comes from ethzasl_ptam (https://github.com/ethz-asl/ethzasl_ptam.git

- ethzasl_ptam : Modified version of Parallel Tracking and Mapping (PTAM) by George Klein. See http://wiki.ros.org/ethzasl_ptam for a detailed overview.

Code changes added for running on top of Intel-Stratix10 SOC Development Board.
- applied the mmap() based IPC btw HPS and FPGA.

This is the master branch for running on HPS only.
There'll be a sub branch for HPS-FPGA Parallel Processing mode.

[Updated on Jul/16th]
Code changes applied by modified memory map. 
Handling SmallBlurryImage will be implemented for parallel processing by this week.

[Updated on Jul/20th]
Modified the memory map by using multiple memory mapped access.
YS.Shon has verified on Stratix10 board.

[Updated on Jul/21st]
Code changes applied to use small image given by FPGA.
The image is stored into aLevel[4].im. Hence SW' halfSampling is not required in SmallBlurryImage creation.

[Updated on Jul/27st]
Changes appiled into MakeFiles of Third party library. This makes the aarch64 mode support enabled as well as compilation error when used the default git master code.
