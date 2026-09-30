# cross-compiler
This will allow you to compile target application code on your host machine.
- you can install and use it in the our system environment or in docker container
## cross-compiler in host system environment
### installation 
- The cross-compiler setup script is present in the Vitis-
AI repository itself.
- by default, cross compiler will be installed in ~/petalinux_sdk_2022.2 

```bash
#clone Vitis-AI repo,skip the step if you already done it
git clone https://github.com/Xilinx/Vitis-AI

#change directory to find the cross-compiler installer
cd Vitis-AI/board_setup/mpsoc

#change permissions for executing
sudo chmod u+r+x host_cross_compiler_setup.sh

#run installer
./host_cross_compiler_setup.sh
```
`Issues` : Vitis-AI/board_setup/mpsoc: No such file or directory

`Solution`: git clone only cloned the latest version of Vitis-AI repo (which was 3.5).The cross-compiler setup script is not present in this 
repository. switch git branch to 3.0. version to get it 

```bash
git switch 3.0
```
### How to use

- source the cross-compiler when ever you use new terminal interface,this will add usefull compiler flags and paths into system environment
```bash
source /home/user/petalinux_sdk_2022.2/environment-setup-cortexa72-cortexa53-xilinx-linux

# if the above command fails,try
unset LD_LIBRARY_PATH
source /home/user/petalinux_sdk_2022.2/environment-setup-cortexa72-cortexa53-xilinx-linux
```
- go to the desired application folder 
- it contian a `makefile` building the project and shell script `build.sh` contain command to execute makefile
- you can either run the build.sh or execute the command make all to build the appication

### example usage
- we are going to build a sample vision-AI application from vitis library named classification as an example, execute the following command.
```bash
cd ~/Vitis-AI/examples/vai_library/samples/classification
bash -x <requirted directory> build.sh
```
`error`: bash: –x: No such file or directory

`solution`: provide a specific build directory or remove -x to build into the current directory

## cross-compiler inside docker
### installlation (not sure)
- If installing the cross-compiler inside the Docker container, then we need to do it every time we run docker
. Need to see if there is a way to save the changes to the container image.
- run our docker container
```bash
cd <Vitis-AI_install_path>/Vitis-AI

./docker_run.sh xilinx/vitis-ai-<pytorch|opt-pytorch|tensorflow2|opt-tensorflow2|tensorflow>-<cpu|rocm>:latest
```
- Activate the vitis-ai-pytorch conda environment
```bash
conda activate vitis-ai-pytorch
```
then 
```bash
cd /workspace/board_setup/mpsoc
./host_cross_compiler_setup.sh
```
#### note:
- found two compiler installation files dont know which one to use
    1. vitis-AI>>board setup>>installer
    2. vitis-AI>> docker>>common>>installer
### how to use(not workings)
- run our docker container
- source our Cross Compiler
```bash
source /home/vitis-ai-user/petalinux_sdk_2022.2/environment-setup-cortexa72-cortexa53-xilinx-linux

#If you run the above command failed,
unset LD_LIBRARY_PATH
source /home/vitis-ai-user/petalinux_sdk_2022.2/environment-setup-cortexa72-cortexa53-xilinx-linux

```
```bash
#note 
you should source the cross-compiler when ever you use new terminal interface. Also, if you forget to run this command outside of Docker, you can execute it later from within.
```
- go to the desired application folder 
- either run the `build.sh` or execute the command `make all` to build the appication
```bash
#why we can`t run compiler inside docker
- may be a corrupted vitis-ai repo
- it is showing unable to find header files while compiling apps
    - this might be becuase- usually headerfile paths are added to system environment by sourcing cross compiler
    - if we are sourcing outside the Docker container will not automatically inherit the environment variables and paths set in your terminal session unless you explicitly pass them when starting the container.
    - tried sourcing inside docker using the same command using outside docker.but since docker cant find the sourcing files,it is not working
- find where is petalinux sourcing in docker
- try reinstalling repo
- try switching to latest branch
- found two compiler installation files
    1. vitis-AI>>board setup>>installer
    2. vitis-AI>> docker>>common>>installer
```

## Important Observation getting errors while compiling the app with in docker
The process as per the documentation is to install the cross-compiler and source it before starting the Docker container. But, this way of doing it cause problem. 
- If getting compilation error repeatedly, it can imply a corrupted Vitis-AI repository.
- As a solution, clone the Vitis-AI repository freshly. Don’t install it on the host machine.
- Run the Docker container, activate the appropriate conda environment, then install the cross compiler,
- then execute the source command shown at the end of the installation logs. Do “unset LD_LIBRARY_PATH” ONLY if there is error stating “Your environment is misconfigured”.

Note: Instead of cloning the Vitis-AI repository again, alternatively we can backup the Vitis-AI repository (create a copy of the /home/user/Vitis-AI folder), and restore it to start freshly.

- in makefile for building application some header file paths are missing 
    - /opt/petalinux/2021.1/sysroots/cortexa72-cortexa53-xilinx-linux/install/Release/lib/
    - /install/Release/include
- it seems like makfile is asking for cross compiler 2021.1 we installed 22.1

## notes
- it seems like the xilings using the following compilers
    - aarch64-xilinx-linux-gcc
    - aarch64-xilinx-linux-g++
    - aarch64-xilinx-linux-ld
