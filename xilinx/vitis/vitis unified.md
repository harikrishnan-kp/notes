<table class="sphinxhide" width="100%">
 <tr>
   <td align="center"><img src="https://raw.githubusercontent.com/Xilinx/Image-Collateral/main/xilinx-logo.png" width="30%"/><h1>Vitis™ unified software platform</h1>
   <a href="https://www.xilinx.com/products/design-tools/vitis.html">See Vitis™ Development Environment on xilinx.com</a>  </td>
 </tr>
</table>
The Vitis™ unified software platform enables the development of embedded software and accelerated applications on heterogeneous Xilinx® platforms including FPGAs, SoCs, and Versal™ ACAPs.


## This kit includes the following tools & libraries
- `Vitis` (previously xilinx sdk) is an integrated development environment (IDE) based on eclips. 
- [`vitis HLS`](vitis%20HLS.md): High Level Synthesis – develop hardware acceleration on FPGA using C, C++, or OpenCL instead of Verilog/VHDL.
-  `Vitis Embedded` – For developing C/C++  application code running on embedded Arm processors
- [`Vitis AI`](vitis%20AI) – Optimized for AI/ML acceleration on Xilinx platforms.
- Integration with Xilinx Vivado – For FPGA logic design.
- `Vivado® Design Suite` for implementing the kernel on the target device, and for developing custom hardware platforms.
- `Vitis Model Composer`: A model-based design tool that enables rapid design exploration within the MathWorks Simulink® environment
- `Vitis Analyzer`
- `Compiler and simulators` – For implementing designs using the AI Engine array
- A set of open-source, performance-optimized library functions, such as DSP, Vision, Solver, Ultrasound, BLAS, and many more, that can be implemented in FPGA fabric or using AI Engines
- vitis allow flashing our program via serial port
    
## reference links
- vitis github: https://github.com/Xilinx/kria-vitis-platforms
- vitis tutorial: https://github.com/Xilinx/Vitis-Tutorials/tree/2024.2?tab=readme-ov-file#tutorials



## Installation
- download the vitis `installation package`: https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis/2024-2.html  
  - The best approach is to downlaod the full package, if you are choosing web installer, you may need to download the entire files from start if there is an error(although there is a resume option) also the download process is very slow
- vitis `installation guide` (choose vitis version from the top side) : https://docs.amd.com/v/u/en-US/ug1393-vitis-application-acceleration
- check installation guide for OS compatability,
  - `only vitis version 24.2 support ubuntu 24.04`
  - `vitis version 22.2 and above support ubuntu 22.4`

### installing vitis 22.1 on ubuntu 24(not web installer)
- install required dependencies by checking installation guide
- install libtinfo5, check installation error section
- extract and run the installer(xsetup)
  - Customize your installation by selecting design tools and devices 
- After a successful installation run installLibs.sh
```bash
cd <install_dir>/Vitis/<release>/scripts
#allow permission if needed
sudo ./installLibs.sh
```

### installation errors
- installation stuck at “Generating installed device list”.
  - This could be due to an unsupported OS
  - check the installation log to see an actual reason why the installation got stuck
  ```bash
  #installation log can be found at
  cd ~/.Xilinx/xinstall
  ```
  - Ubuntu 24.04 provides `libtinfo6`, but the vitis 22.1 installer needs `libtinfo5`. The solution is to install libtinfo5 before running the installer (libtinfo5 is not available in ubuntu 24 apt)
  ```bash
  sudo curl -O http://launchpadlibrarian.net/648013231/libtinfo5_6.4-2_amd64.deb

  sudo dpkg -i libtinfo5_6.4-2_amd64.deb
  ```
  - some other packages found for installing if libtinfo5 didn`t work (use apt or other package managers to download)
    - libtinfo-dev
    - lib32stdc++6
    - libgtk2.0-0:i386
    - libfontconfig1:i386
    - libx11-6:i386
    - libxext6:i386 
    - libxrender1:i386
    - libsm6:i386

  - another solution found: https://adaptivesupport.amd.com/s/question/0D52E00006hpQNASA2/vivado-installation-got-stuck-says-generating-installed-devices-list?language=en_US
## use
- setup environment(if you are not using gui interface(need to verify))
```
source <Vitis_install_path>/Vitis/<version>/settings64.sh
``` 
