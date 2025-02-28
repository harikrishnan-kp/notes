# Linux
## What is a Device Tree?
The Device Tree (DT) describes hardware components to the Linux kernel.
Device Tree File Structure
- Source file: `.dts` (Device Tree Source)
- Compiled file: `.dtb` (Device Tree Blob)
- Overlay file: `.dtbo` (Device Tree Blob Overlay)
## Why Use Device Tree Overlays?
- Modify hardware configuration without recompiling the full Device Tree.
- Enable or disable peripherals dynamically (e.g., GPIOs, I2C, SPI).
- we can use the same for adding accelerated hardware(PL)
## Sysroot in Linux
In a Linux system, a sysroot is simply a directory that mimics the root filesystem (/) of the target system