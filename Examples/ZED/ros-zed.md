# ROS-ZED
This doc outlines the installation process for the ZED SDK and ZED ROS package. 

# CUDA Installation 
https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=20.04&target_type=deb_network

# SDK Installation 
Reference: https://www.stereolabs.com/docs/installation/linux/

Download and install the appropriate SDK based on your cuda version.
```
# Check cude version
nvidia-smi

# SDK downloads page
https://www.stereolabs.com/developers/release/

# Make intaller executable
cd {download-path}
chmod +x ZED_SDK_Ubuntu18_v3.0.run

# Run the installer
./ZED_SDK_Ubuntu18_v3.0.run

# Notes
# - Accept the EULA
# - The below output is desireable, CUDA install should get picked up by the SDK installer
Verifying archive integrity...  100%   MD5 checksums are OK. All good.
Uncompressing 'ZED camera SDK by Stereolabs'  100%  
Ubuntu version 20.04 detected. OK
To continue you have to accept the EULA. Accept  [Y/n] ?Y
Installing...
Installation path: /usr/local/zed
Checking CUDA version...
OK: Found CUDA 11.7

# Note from installation regarding python environments
The ZED Python API was installed for 'python3', when using conda environement or virtualenv, the ZED Python API may need to be resetup to be available (using 'python /usr/local/zed/get_python_api.py')
```
