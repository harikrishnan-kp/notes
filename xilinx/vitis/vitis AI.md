<table class="sphinxhide">
 <tr>
   <td align="center"><img src="https://raw.githubusercontent.com/Xilinx/Image-Collateral/main/xilinx-logo.png" width="30%"/><h1>Vitis AI</h1>
   </td>
 </tr>
</table>


AMD Vitis™ AI software is an AI inference development platform for AMD devices, boards, and Alveo™ data center acceleration cards.
It consists of 
- a rich set of AI models : Pready-to-use AI models for various use cases(eg. for object detection, image classification, and natural language processing). Developers don’t need to train models from scratch,just optimize and deploy.
- an optimized neural network processing unit (NPU) core(RTL implementations): which can be scaled and deployed on Xilinx FPGAs and MPSoCs for hardware accelerated inferencing
- tools : Provides a user-friendly AI toolchain that simplifies the process of compiling, quantizing, and deploying AI models. it support popular AI frameworks like TensorFlow, PyTorch, and ONNX.
- libraries and example designs.

It is designed with high efficiency and ease of use in mind, empowering developers to unleash the full potential of AI acceleration.

- [github repo](https://github.com/Xilinx/Vitis-AI)
- [Documentation of Vitis-AI V3.0](https://xilinx.github.io/Vitis-AI/3.0/html/index.html)
- [Userguide of vitis-AI 3.0](https://docs.amd.com/r/3.0-English/ug1414-vitis-ai)

## Installing Vitis V3.0
installation guide: https://xilinx.github.io/Vitis-AI/3.0/html/docs/install/install.html#

old xilinx doc: https://docs.amd.com/r/3.0-English/ug1354-xilinx-ai-sdk/Vitis-AI-Library-File-Locations
## Vitis AI installation errors
- vitis AI repo clone failure : may be because of the the large size of repo or slow network
```
error: RPC failed; curl 92 HTTP/2 stream 5 was not closed cleanly: CANCEL (err 8)
error: 20171 bytes of body are still expected
fetch-pack: unexpected disconnect while reading sideband packet
fatal: early EOF
fatal: fetch-pack: invalid index-pack output
```
solution : https://github.com/desktop/desktop/issues/19067 
- permission issues while pulling pre-built docker container
```
permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Post "http://%2Fvar%2Frun%2Fdocker.sock/v1.47/images/create?fromImage=xilinx%2Fvitis-ai-pytorch-cpu&tag=latest": dial unix /var/run/docker.sock: connect: permission denied
```
Solution : add the current user to the docker group so they can run Docker commands without using sudo.
```bash
sudo usermod -a -G docker $USER
newgrp docker

#verify the change by running 
docker run hello-world
```
## Run Vitis-AI docker container
move to vitis-Ai repo and run required docker container 
 
```bash
#in our case
./docker_run.sh xilinx/vitis-ai-pytorch-cpu:latest
```
## Setup kv260 for Vitis-AI
https://xilinx.github.io/Vitis-AI/3.0/html/docs/quickstart/mpsoc.html#setup-the-target
- flash [prebuild vitis-AI OS](https://www.xilinx.com/member/forms/download/design-license-xef.html?filename=xilinx-kv260-dpu-v2022.2-v3.0.0.img.gz) for kv260 to sd card using flashing tools (belena etcher)   
- password: root
- connect to kv260 using serial communication tool or ssh

## Run a vision-AI app on kv260
- setup target-kv260: OS installation, commincation setup and other things
- build vision-AI application 
  - use prebuild application or
  - build from our computer (cross-compile) or
  - build from target (kv260) 
- Setup AI model for inferencing
  - use pre build model or
  - Quantize and compile the required AI model for running on kv260 using Vitis AI
   - copy the model file `<AI-model>.xmodel` and Vitis™ AI configuration file `<AI-model>.prototxt` to `/usr/share/vitis_ai_library/models/<AI-model>`
- setup input test images and videos to ensure our vision AI application is working on kV260
  - test [imgs & videos](https://www.xilinx.com/bin/public/openDownload?filename=vitis_ai_runtime_r3.0.0_image_video.tar.gz) for example apps in `vai_runtime`
  - input samples [img](https://www.xilinx.com/bin/public/openDownload?filename=vitis_ai_library_r3.0.0_images.tar.gz) & [videos](https://www.xilinx.com/bin/public/openDownload?filename=vitis_ai_library_r3.0.0_video.tar.gz) for example apps in `vai_library`
```bash
#note: if you are using vitis-AI OS image in our kv260 it contain prebuild apps,models and test inputs
```
- run the app by providing test inputs
```bash
#change to directory where the application is present before execution
./<executable_file_of_app> <Ai_model> <img/video_sample_input_path> 
```
## Run a pytorch AI model on kv260 using vitis AI
[pytorch tutorials](https://xilinx.github.io/Vitis-AI/3.0/html/docs/quickstart/mpsoc.html#pytorch-tutorial)

- this include model quantization
- cross compiling for specific DPU 
- model deployment and run on kv260

## note
- `/workspace` directory in Docker corresponds to your `/Vitis-AI`directory on the host. That is 
  ```bash
  [Docker] /workspace/examples/vai_runtime/resnet50_pt = [Host] Vitis-AI/examples/vaiexamples/vai_runtime/resnet50_pt.
  ```
- to run an AI model on kv260, it should be converted into `<model_name>.xmodel `
- `.xmodel` is the AI model quantized an compiled for running on kv260 or other xilixs dev boards
- The `.prototxt` file is a Vitis™ AI configuration file that facilitates the uniform configuration management of model parameters. Please refer to the Vitis AI User Guide to learn more.

# doubt
- what is the difference between vai_library and vai_runtime examples
- why compiling example apps failing inside docker container(cross compilation)
- does the prebuild vitis-AI OS came with a DPU(for running on PL),if it is where the DPU realetd file are located

