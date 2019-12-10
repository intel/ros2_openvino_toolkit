# Development and Target Platform

>> The development and target platforms have the same requirements, but you can select different components during the installation, based on your intended use.

## Hardware
### Processor Supported:
- Intel architecture processor, e.g. 6th~8th generation Intel® Core™
- Intel® Xeon® v5 family
- Intel® Xeon® v6 family
- Intel® Pentium® processor N4200/5, N3350/5, N3450/5 with Intel® HD Graphics

**Notes**:
- Processor graphics are not included in all processors. See [Product Specifications](https://ark.intel.com/) for information about your processor.
- A chipset that supports processor graphics is required for Intel® Xeon® processors.
- Use one of the following methods to determine the GPU on your hardware:
	* [lspci] command: GPU info may lie in the [VGA compatible controller] line.
	* Ubuntu system: Menu [System Settings] --> [Details] may help you find the graphics information.
	* Openvino: Download the install package, install_GUI.sh inside will check the GPU information before installation.

### Pripheral Depended:
- Intel® Movidius™ Neural Compute Stick
- Intel® Neural Compute Stick 2
- Intel® Vision Accelerator Design with Intel® Movidius™ VPU
- RGB Camera, e.g. RealSense D400 Series or standard USB camera

## Operating Systems
- Ubuntu 16.04 or 18.04 long-term support (LTS), 64-bit: Minimum supported kernel is 4.14
- CentOS 7.4, 64-bit (for target only)
- Yocto Project Poky Jethro v2.0.3, 64-bit (for target only and requires modifications)

**Note**: Since **Ubuntu 18.04** in the list is the only one well supported by ROS2 core, it is highly recommended to use as the OS.
