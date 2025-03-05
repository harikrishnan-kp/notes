# steriolbm
https://github.com/Xilinx/Vitis_Libraries/tree/main/vision

- contain 3 levels of examples
- go to L2 
- input images can be found at data folder

# The Vitis library
The Vitis library is organized into L1, L2, and L3 folders to facilitate various development stages.

L1: Makefiles and sources in L1/examples and L1/tests facilitate HLS-based flows for a quick check without considering the complexities of Platform, OpenCL/XRT, or framework. the following list shows the various uses:

L2 : Makefiles and sources in L2/examples and L2/tests facilitate building XCLBIN file from various sources (HDL, HLS or XO files) of kernels with host code written in OpenCL/XRT frame work targeting a device. This flow supports:

L3 : Makefiles and sources in L3/examples and L3/tests demonstrate applications developed using multiple kernels in the pipeline. The Makefiles provided can be used for executing tasks same as L2.

## make error
- `PLATFORM` variable is exported by us,it is `kv260_custom.xpfm`
- `PLATFORM_NAME` = kv260_custom
- `XPLATFORM` definition can be found at utils.mk,line 74