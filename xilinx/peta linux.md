# PetaLinux
- PetaLinux is an embedded Linux development platform designed specifically for Xilinx SoCs (System-on-Chip) and FPGAs (like the Kria KV260).
- PetaLinux SDK tool contains everything necessary to build, develop, test, and deploy embedded Linux systems.
- The linux distro getting from petalinux tool is generally mentioned as petalinux
- peta linux tool is usefull when we want to build a custom OS that can accomodate new IP(need clarification)
- peta linux tool contain the following 
    - Yocto Extensible SDK (eSDK)
    - XSCT and toolchains
    - PetaLinux Command Line Interface (CLI) tools
- For all the embedded software applications configuration, PetaLinux tool will use the XSCT underneath.
- PetaLinux Board Support Packages (BSP) include pre-built images and a pre-defined configuration to rebuild the images from scratch. BSPs only support a single static HW configuration
```bash
# important note
here im going to work with petalinux 2022.1, because im using all other xilinx tools(like vitis,vivado) version  2022.1
```

## installation
installation quide:https://docs.amd.com/r/2021.1-English/ug1144-petalinux-tools-reference-guide/Installation-Steps
- petalinux 22.1 only support ubuntu 20 and below (lets try anyway)
- PetaLinux 2021.1 works only with hardware designs exported from Vivado® Design Suite 2021.1.
- download petalinux installer
https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/embedded-design-tools.html
- install it: `do not install into root directories`- this will overwrite files critical to your Linux system's operation (worst case)
```bash
chmod 755 ./petalinux-v<petalinux-version>-final-installer.run

./petalinux-v<petalinux-version>-final-installer.run --dir <installation_path>> --platform "aarch64"
```
- aarch64 refers to Zynq UltraScale+ MPSoC
- install gawk and other packages if needed
- installation error : even after installing the required package
    ```
    ERROR: You are missing these development libraries required by PetaLinux:
    zlib1g:i386
    ```
    - solution:https://adaptivesupport.amd.com/s/question/0D54U00008ShJduSAF/error-you-are-missing-these-development-libraries-zlib1gi386-even-though-is-installed?language=en_US

## use
- `did not understand` PetaLinux tools require that your host system /bin/sh is 'bash'. If you are using Ubuntu distribution and your /bin/sh is 'dash', consult your system administrator to change your default system shell /bin/sh with the sudo dpkg-reconfigure dash command.
- dash scripts should generally run without issues in bash...But, not vice versa
- change default shell from dash to bash
- - source PetaLinux settings

## building OS for kv260 using xilinx provide BSP
- downlaod bsp 

```
petalinux-util --webtalk on
```
## reference links
- petalinux bsp : https://xilinx-wiki.atlassian.net/wiki/spaces/A/pages/1641152513/Kria+SOMs+Starter+Kits#PetaLinux-Board-Support-Packages
- petalinux user guide: https://docs.amd.com/r/2022.1-English/ug1144-petalinux-tools-reference-guide/Introduction
- how to create a custom embedded Linux image for the Kria KV260 Vision AI Starter Kit in PetaLinux 2021.1 : https://www.hackster.io/whitney-knitter/getting-started-with-the-kria-kv260-in-petalinux-2021-1-b491fd
- how you can customize a PetaLinux project to work for Vitis acceleration applications: https://docs.amd.com/r/lD8AuK8H9fr8ygi_oDV_8Q/7fGeKvfh1jEQwMT0xribMQ