<table class="sphinxhide" width="100%">
 <tr>
   <td align="center"><img src="https://raw.githubusercontent.com/Xilinx/Image-Collateral/main/xilinx-logo.png" width="30%"/><h1>Vitis™ Embedded Software Development Platform</h1>
   <a href="https://www.xilinx.com/products/design-tools/vitis.html">See Vitis™ Development Environment on xilinx.com</a>  </td>
 </tr>
</table>

Xilinx Vitis (previously xilinx sdk) is an integrated development environment (IDE) based on eclips designed for heterogeneous computing on Xilinx FPGAs, SoCs, and ACAPs. It allows developers to program Xilinx hardware using high-level languages (C, C++, and OpenCL)

## Features of Vitis
- `High-Level Synthesis (HLS)` – develop hardware acceleration on FPGA using C, C++, or OpenCL intead of Verilog/VHDL.
- Embedded software development for PS and PL 
- `Vitis AI` – Optimized for AI/ML acceleration on Xilinx platforms.
- Integration with Xilinx Vivado – For FPGA logic design.
- `Vivado® Design Suite` for implementing the kernel on the target device, and for developing custom hardware platforms.
- `Vitis Model Composer`
- vitis allow flashing our program via serial port
## Notes
- vitis need a hardware desciption file (XSA - Xilinx Support Archive) for creating a platform specific applications
- if we are using PS only for develoment we need the XSA of our PS for creating our development platform
- in vitis 2021.1 XSA for kv260 PS is not available, so we need to downlaod it from external source
- To install a specific platform XSA, download the zip file and extract it into /opt/xilinx/platforms, or extract it into a separate location and add that location to the PLATFORM_REPO_PATHS environment variable.
- if we want to use both PS and PL we should have the specific pre build XSA from vivado 
- This XSA file contains:
  - FPGA bitstream
  - Memory map (AXI base addresses, IP registers)
  - Device tree information for Linux
- Vitis uses the .XSA file to generate Provides drivers and APIs to control FPGA IPs
- If you're working with AI acceleration on `Vitis AI` can be used to optimizes models for DPU (Deep Learning Processing Unit- on PL) on Kria K26. it Supports TensorFlow, PyTorch, ONNX, etc.
- `Vitis HLS` - it automatically converts the C/C++ code into an RTL design (Verilog/VHDL), which describes how the hardware should function.
- while hardware accelerated kernels are compiled into an executable device binary `.xclbin`
## Installation 
- The best approach is to install `Vitis Core Development Kit` that combines all aspects of Xilinx® software development into one unified environment.
- it includes Vivado® Design Suite, Vitis Model Composer, Vitis HLS and software development tool stack, such as compilers and cross-compilers, to build your host program and kernel code, analyzers, debugger. There is no need to install Vivado separately.
- vitis installation guide (choose vitis version from the top side) : https://docs.amd.com/v/u/en-US/ug1393-vitis-application-acceleration
- check installation guide for OS compatability,
  - `only vitis version 24.2 support ubuntu 24.04`
  - `vitis version 22.2 and above support ubuntu 22.4`
- it is suggested to download required packages and manually install it,it allows to reuse the downloaded package incase of any installation errors   
- download vitis: https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vitis/2024-2.html  
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
  - some other packages found to be good for installing if needed (use apt or other package managers to download)
    - libtinfo-dev
    - lib32stdc++6
    - libgtk2.0-0:i386
    - libfontconfig1:i386
    - libx11-6:i386
    - libxext6:i386 
    - libxrender1:i386
    - libsm6:i386

  - another solution found: https://adaptivesupport.amd.com/s/question/0D52E00006hpQNASA2/vivado-installation-got-stuck-says-generating-installed-devices-list?language=en_US
  - found the following warning while installing vitis 22.1,24.2 on ubuntu 24 using self extracting web installer
  ```bash
  Verifying archive integrity... All good.
  Uncompressing Xilinx Installer..................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................
  This is a fresh install.

  INFO  - Started in: 2 Sec 

  INFO  - Internet connection validated, can connect to internet. 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  WARN  - MissingBundle for: DEFAULT_DESTINATION_FOLDER_LIN_DownloadSelectedProd, will use the key as the value 

  INFO  - Download image directory is /home/user/Downloads/xlnx_dwnld_pkg 

  ERROR - Checksum failed, expected: 889161cf7a169c41ef7b6032fdd6a1c3 but was: 7ba83f98fc1ef777f261638b6be5d1c1 for file https://amd-ax-dl.entitlenow.com/dl/ul/2022/04/20/R210580947/rdi_0890_2022.1_0420_0327.xz?hash=SvQmN60YnJ428R-O-WBGTw&expires=1739541734&filename=rdi_0890_2022.1_0420_0327.xz&sessionid=060dd96a-f69c-4925-b035-a84444325ca5 
  ```
## Vitis embedded software development flow
The Vitis application acceleration development flow provides a framework for developing and delivering FPGA accelerated applications using standard programming languages for both software and hardware components. The software component, or host program, is developed using C/C++ to run on x86 or embedded processors, with OpenCL™ API calls to manage runtime interactions with the accelerator. The hardware component, or kernel, can be developed using C/C++, OpenCL C, or RTL. The Vitis software platform promotes concurrent development and test of the Hardware and Software elements of an heterogeneous application.
## Bare metal build methods
This section describes how to create a sample Hello World application using an existing template.
- Launch the Vitis software platform.
- Select a workspace directory for your first project.
Click Launch. The welcome page appears.
- Close the welcome page.
- Select File > New > Application Project.
Type a project name into the Project Name field.
- Select the location for the project. You can use the default location as displayed in the Location field by leaving the Use default location check box selected. Otherwise, click the check box and type or browse - to the directory location.
- Select Create a new platform from hardware (XSA). The Vitis IDE lists all the available predefined hardware designs.
- Select any one hardware design from the list and click Next.
- From the CPU drop-down list, select the processor for which you want to build the application. This is an important step when there are multiple processors in your design. In this case you can either select psu_cortexa53_0 or psu_cortexr5_0.
- Select your preferred language: C or C++.
- Select an OS for the targeted application.
- Click Next to advance to the Templates screen.
- Select Hello World and click Next. Your workspace opens with the Explorer pane showing the hello_world_system system project and the KV260 platform project.
- Right-click the system project and select Build Project. You have now built your application and the Console tab shows the details of the file and application size.

Now that you have generated the executable binary, you can test it on a board. To run the application on the board, perform the following preliminary steps: 
- Connect a JTAG cable to the computer.
- Set the Boot Mode switch of the board to JTAG mode. 
- Connect a USB UART cable and setup your UART console. Power up the board. 
- Expand the system project and choose the application project you want to debug.
- Right-click the application and select Debug As → Launch on Hardware (Single Application Debug).
- On the Confirm Perspective Switch dialog, click Yes. The Vitis IDE switches to the Debug perspective and the debugger stops at the entry to your main() function. 
- Using the commands in the toolbar, step through the application. - After you step through the print() function, Hello World appears in the UART console

## reference
* vitis github: https://github.com/Xilinx/kria-vitis-platforms


## doubts
* are we creating general purpose applications or application for PS-PL communucation using vitis?
    - vitis can be Used for software development on the ARM Cortex-A53 cores (PS), including Linux applications, bare-metal firmware, and managing PS-PL communication.
* if we are using vitis for general purpose application development,why can't with other IDEs?
* Why Do We Feed the Vivado Bitstream to Vitis Before Development?
    - The reason for this is to ensure that the Processing System (PS) correctly recognizes and interacts with the Programmable Logic (PL).
    - Vitis uses the .xsa file to generate drivers and software APIs.
* what modifications are need to our OS if we add an hardware accelerator


