# Linux
## What is a Device Tree?
devicetree (also written device tree) is a data structure describing the hardware components of a particular computer so that the operating system's kernel can use and manage those components, including the CPU or CPUs, the memory, the buses and the integrated peripherals. 

Once a device tree source file (`.dts`) is created, it needs to be compiled into a binary format called a device tree blob (`.dtb`). The device tree compiler (`dtc`) is used for this purpose. resulting a `.dtb` file

device tree blob location changes with different system 
- /lib/firmware/<kernel-version>/device-tree/
- /boot/dtbs/<kernel-version>/<board>.dtb

When an embedded system boots up with a Linux kernel, the bootloader (such as U-Boot) loads the device tree blob into memory and passes its address to the kernel as a command-line argument. The kernel then parses the device tree and uses the information to configure itself and initialize the necessary drivers for the hardware components described in the device tree.


This dynamic configuration allows the same Linux kernel image to work on different hardware platforms without the need for recompilation. It provides a level of hardware abstraction and portability, making it easier to support multiple board variants or even entirely different systems with the same kernel.

https://octavosystems.com/linux-device-trees-embedded-systems/

### device tree overlay
An overlay is a separate device tree file that can be applied on top of the base device tree, adding or modifying nodes and properties.

Overlays are useful when you have a base board with a fixed device tree but want to add support for optional or custom hardware modules. By applying an overlay, you can dynamically modify the device tree at runtime without the need to recompile the entire device tree source.

#### Use
- Modify hardware configuration without recompiling the full Device Tree.
- Enable or disable peripherals dynamically (e.g., GPIOs, I2C, SPI).
- we can use the same for adding accelerated hardware(PL)

Device Tree File Structure
- Source file: `.dts` (Device Tree Source)
- Compiled file: `.dtb` (Device Tree Blob)
- Overlay file: `.dtbo` (Device Tree Blob Overlay)
 

## The root file system (rootfs)
In Linux, directories are structured in a tree-like hierarchy.this directory structuer is called as root file system(rootfs).The rootfs is the fundamental component of a Linux system, This well-defined directory structure provides easy isolation and identification of data.The root is at the top of the hierarchical file tree (also known as ‘/’).

### Contents of rootfs
1. `/bin`: The /bin directory contains binaries of essential user commands that can be used by all users of the system. Here we can find ready-to-execute Linux commands like ls, cp, and echo, and shells like bash and csh. Moreover, /bin also contains the ready-to-run executables that may be required in order to boot or repair a system.

2. `/boot`: In the /boot directory we can find files used for booting an operating system, usually kernel images, bootloader (for example LILO, GRUB) related files, configuration files, and map installer. Generally, we can find vmlinuz, System.map, initrd, and config files here. Among them, vmlinuz is our actual kernel. The bootloader loads this kernel and executes it. The System.map file has a list of kernel symbols and respective addresses. The initial ramdisk (initrd) file is useful in loading drivers that are necessary for the initial booting process. The config file contains a list of kernel options, a list of devices, and device options.

3. `/dev`: The /dev directory is where all device files are stored. The device files residing in this directory represent devices attached to our system. Applications can interact with them using system calls to perform I/O operations. The majority of the device drivers that correspond to the hardware devices are one of two types: block device drivers or character device drivers.

4. `/etc`: Configuration files specific to the local machine are contained in the /etc directory. This directory holds most global config files. However, larger software packages can have their own subdirectories under /etc. Some examples are /etc/X11, /etc/sgml, and /etc/xml.

5. `/home`: The /home directory is the default location to create home directories for different users. For example, let’s say there’s a user with the username “Sam“. Then, the default home directory of Sam will be /home/Sam. In other words, directories under /home provide a personal workspace to regular users who don’t have root privileges.

6. `/lib`: The /lib directory contains essential shared libraries for system boot. Device drivers necessary for system boot are placed under subdirectory /lib/modules/’kernel-version’. It also contains the libraries needed by binaries from /bin and /sbin to run the commands in the root filesystem.

7. `/media`: The /media directory contains subdirectories that are utilized as mount points when we connect any removable media devices to the system. We can find subdirectories or symbolic links to directories representing different removable media devices like CD-ROMs and USB sticks. For example, on inserting a CD into our Linux system, a directory will automatically be created inside the /media directory. We can use this to access the contents of the CD inside this directory.

8. `/mnt`: The /mnt directory is a mount point where we can mount a temporary filesystem that resides on a storage device like a hard-disk drive, USB stick, or CD-ROM. Unlike /media, where the system mounts removable media automatically, under /mnt we need to mount manually. This directory can be empty or may have subdirectories to mount individual devices.

9. `/opt`: The /opt directory contains optional software packages. In this context, optional software packages are those that are not part of the system — for instance, third-party software that we install as add-ons.

10. `/root`: The /root directory is the home directory of the root user of the system.

11. `/run`: The /run directory stores the system information data describing the system since its booting. Applications store their transient files like process IDs, socket descriptors, and more in this directory.

12. `/sbin`: Similar to the /bin directory, the /sbin (system binaries) directory contains ready-to-run executables needed to boot our Linux system. However, unlike /bin binaries, /sbin contents are intended to be executed by the root user.

13. `/srv`: The /srv directory has service data. In other words, site-specific data served by our system is likely to be stored here. For instance, if we’re using an HTTP server to serve a website, then we may store files related to our website inside this directory.

14. `/tmp`: The /tmp directory contains files that are temporary. Many of these files are created by currently running processes on our system that store their temporary data under this directory. Therefore, a clearing out of this directory may happen at booting or at system shutdown.

15. `/usr`: The /usr directory contains executables and read-only data. In contrast with files that are used by the system, files under this directory are used by users. So, /usr/sbin and /usr/bin directories contain non-essential /bin and /sbin binaries, respectively. The default installation location for locally compiled applications is under /usr/local. 

16. `/var`: The /var (variable) directory contains transient files and temporary files whose size may change. We can find a number of spools and log files here.

## Sysroot
- when we are compiling an application to run on Linux system, a sysroot is simply a directory that mimics the root filesystem (/) of the target system for the purpose of locating headers and libraries
- linux os prefer dynamic linking of library(ie,we are using libraries on runtime) to reduce memory usage and updating easier. it provide of bundles of commonly used dynamic libaries
- if we are cross compiling an application with dynamic linking to run on linux OS,we should know where the dynamic library bundles are present(because os implimentation may vary with embedded device)
- in this case sysroot act as a reference for the cross-compiler
- sysroot is a directory containing a minimal set of system libraries and headers necessary for cross-compiling software for a target system, essentially a scaled-down version of the target system's root filesystem, allowing a compiler on your host machine to build executables for a different target platform by providing the correct header files and libraries to link against. 
- A sysroot typically includes essential directories like /usr/include (for header files) and /usr/lib (for libraries) specific to the target system. 
Compiler flag:
- When compiling code for a target system, you specify the path to the sysroot using the `--sysroot` flag with your compiler.

## Free course embedded linux: 
https://octavosystems.com/courses/getting-started-with-embedded-linux/