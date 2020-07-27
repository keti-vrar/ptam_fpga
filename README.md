ptam_fpga
============

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
