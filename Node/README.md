# Embedded software for igvc comp 1

## Prerequisites
[STM32CubeMx](https://www.st.com/en/development-tools/stm32cubemx.html) needs to be installed to generate the boilerplate STM32 files.
[wsl](https://docs.microsoft.com/en-us/windows/wsl/) is also required if on windows, as it is easier than using mingw for building.
[gcc-arm-none-eabi](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) is the cross-compiler necessary to build for STM32 on linux.
- This can usually be installed using the command `sudo apt install gcc-arm-none-eabi` on Ubuntu (both bare-metal and wsl)

## Build steps
- Open Node.ioc in STM32CubeMx, and hit generate code.
- Run build.sh, which will create a build directory, run cmake, and then run make to build the project.
