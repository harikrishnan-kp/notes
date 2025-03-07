# steriolbm
https://github.com/Xilinx/Vitis_Libraries/tree/main/vision

- ZynqMP common image for compilation can be seen  in petalinux downlaod section: https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/embedded-design-tools/archive.html

- contain 3 levels of examples
- go to L2 >> examples >> stereolbm
- we can see, HLS code for hardware kernal and applicaion 
- input images for app can be found at data folder
- rules to build hardware kernal and app is in makefile,modify makefile to build for our custom platform
- build makefile
- you will get xcbin and application binary
- copy xclbin,app and pl.dtbo to kv260 /lib/firmware/xilinx/<folder name>
- loadapp and run application 


# The Vitis library
The Vitis library is organized into L1, L2, and L3 folders to facilitate various development stages.

L1: Makefiles and sources in L1/examples and L1/tests facilitate HLS-based flows for a quick check without considering the complexities of Platform, OpenCL/XRT, or framework. the following list shows the various uses:

L2 : Makefiles and sources in L2/examples and L2/tests facilitate building XCLBIN file from various sources (HDL, HLS or XO files) of kernels with host code written in OpenCL/XRT frame work targeting a device. This flow supports:

L3 : Makefiles and sources in L3/examples and L3/tests demonstrate applications developed using multiple kernels in the pipeline. The Makefiles provided can be used for executing tasks same as L2.
# TODO
running stereolbm but the os ,doesnt have opencv files,unable to install opencv,beacuse
    - desired version was not avialable in dnf packet manager
    - tried downloading files from git and build, but unsuccessfull due to disk space limitations
    - tried moving required opencv library files(executable) in sysroot(used in cross-compiling)into kv260,but not working  
- are we using the correct platform
    - found some precreated kria platforms on github(but seems like it is  dedicated  apps), is it possible to use these .xpfm file for this project 
        - platforms are hardware kernal specific as different hardware kernals needs different AXI interface,clocks and etc.
        - hardware kernal may change with appication 
    - is it possible to use the same platform.xpfm file created for vadd in stereolbm
        - we can use the same platform, if stereolbm use the same hardware kernal as vadd `OR` if the stereolbm kernal is complatable with the AXI interface and clock designed for vadd
    - got hint that the platform we created is same as platform form ZCU104(need more clarification)
- make sure the modification done on make is correct
    - `PLATFORM` variable is exported by us,it is `kv260_custom.xpfm`
    - `PLATFORM_NAME` = kv260_custom
    - `XPLATFORM` definition can be found at utils.mk,line 74
- in previos compiled app we got opencvlib error,how to overcome this
    - this library is present in the sysroot we used for cross compilation, but not present in the kria os rootfs
    - installed new os on kria,but required opencv lib is not available
    - installed opencv from dnf using `sudo dnf install opencv` 4.5.2 (latest by default)
    - in documentation of they are saying to install opencv 4.4.0 
    - remind that in documentation there is a prerequisite of `opencv-4.4.0-contrib` too
- try different os
    - installed petalinux 22.1 now(need to try)
    - create OS using the rootfs from xilinx commom image in petalinux tool
