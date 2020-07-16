ptam_fpga
============

[Updated on Jul/15th]

The baseline comes from ethzasl_ptam (https://github.com/ethz-asl/ethzasl_ptam.git

- ethzasl_ptam : Modified version of Parallel Tracking and Mapping (PTAM) by George Klein. See http://wiki.ros.org/ethzasl_ptam for a detailed overview.

Code changes added for running on top of Intel-Stratix10 SOC Development Board.
- applied the mmap() based IPC btw HPS and FPGA.

This is the master branch for running on HPS only.
There'll be a sub branch for HPS-FPGA Parallel Processing mode.


