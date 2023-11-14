# Table of content
- [Table of content](#table-of-content)
- [EDScorbot](#edscorbot)
- [Repository folders](#repository-folders)
- [Mosquitto and dependences cross-compilation](#mosquitto-and-dependences-cross-compilation)

# EDScorbot
This repository contains the source code for a spike-based controller for the ED-Scorbot robotics platform. This controller has been implemented for hardware using VHDL rtl languaje. This controller module has been tested and deployed on a Zynq 7100 Xilinx platform called Zynq MMP board. The Rbtanno controller module can be configured from an user application through an AXI-4 module.


# Repository folders
- src: API source code of the EDScorbot controller.
- src_test: Source code for C++ simulation.


# Compiler dependences
This version of EDScorbot runs as MQTT client in the zynq platform. All dependences have to be cross-compiled. For the cross-compilation it will be used the arm-linux-gnueabihf-g++-8 compiler.
- Install the compilers using the following commands:
  ```
  sudo apt-get install g++-8-arm-linux-gnueabihf
  sudo apt-get install gcc-8-arm-linux-gnueabihf
  sudo apt-get install g++-11
  sudo apt-get install gcc-11
  ```

# EDScorbot mqtt app compilation and dependences cross-compilation
Once the dependencies have been compiled successfully, let's move to compile the rbtanno mqtt client:
- In vscode, select compiler: kit GCC 8.4.0 arm-linux-gnueabihf
- Move to the project folder
  ```
  rm -rf lib mosquitto json
  cd build
  rm -rf *
  ```
- Compile the software using Build (All). This will automatically clone mosquitto and json repositories and cross-compile them
  
The resulting software has to be generated in build folder. Copy this app in the zynq platform and launch it.

# Simulation app compilation
Once the dependencies have been compiled successfully, let's move to compile the rbtanno mqtt client:
- In vscode, select compiler: kit GCC 11.1.0 x86_64-linux-gnu
- Move to the project folder
  ```
  rm -rf lib mosquitto json
  cd build
  rm -rf *
  ```
- Compile the software using Build (All) This will automatically clone mosquitto and json repositories and cross-compile them

The resulting software has to be generated in build folder. Launch it.

