### What is Quantization of an AI Model?

Quantization is a process that reduces the precision of numerical values in a deep learning model to make it smaller, faster, and more efficient without significantly sacrificing accuracy.
Why Quantize a Model?

- Reduce Model Size 📉 – Converts high-precision weights (e.g., 32-bit floating-point) to lower-precision (e.g., 8-bit integers), leading to smaller file sizes.
- Increase Inference Speed ⚡ – Reduces computation time, making the model run faster on hardware like microcontrollers, edge devices, and mobile phones.
- Lower Power Consumption 🔋 – Ideal for embedded systems and low-power AI applications.

### what is pytorch 
its an AI framework, extension of model files are `.pth` and `.pt`

### what is hardware and software kernal
A hardware kernel is a specialized function or processing block that runs directly on programmable hardware (such as an FPGA) instead of a traditional CPU. It is used in hardware acceleration to offload compute-intensive tasks from software to hardware, achieving higher performance, lower latency, and better power efficiency. In the Vitis Unified Software Platform, a hardware kernel is a compute function written in C, C++, OpenCL, or RTL (Verilog/VHDL) that gets synthesized into FPGA logic. These kernels execute in parallel and can communicate with a CPU or other kernels via high-speed interfaces like AXI4.

