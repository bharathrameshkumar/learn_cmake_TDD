# learn_cmake_TDD
a repo for me to learn CMAKE and TDD. A move towards vendor free developement env

In STM32CubeIDE, everything was hidden. Here's what was actually happening:
## The compilation pipeline:

> Source (.c) → Preprocessor → Compiler → Object files (.o) → Linker → Executable (.elf) → Binary (.bin/.hex)

### What you need:
- **Compiler**: arm-none-eabi-gcc (cross-compiler for ARM Cortex)
- **Build system**: CMake (generates Makefiles)
- **Debugger**: arm-none-eabi-gdb + OpenOCD or STLink
- **Testing framework**: Ceedling (Unity + CMock + build automation)

## Why TDD for Motor Control?

TDD catches these in minutes:
- **Hardware abstraction**: Test FOC algorithms without hardware
- **Safety critical**: Motor control can destroy hardware or injure people
- **Complex math**: Park/Clarke transforms, PID loops need verification
- **Portability**: Tests prove your code works across platforms

## Project Structure

Here's the structure I use for this project:
```
motor_control_project/
├── CMakeLists.txt              # Root build configuration
├── src/
│   ├── main.c
│   ├── motor_control/
│   │   ├── foc.c              # Field-Oriented Control
│   │   ├── pid.c              # PID controllers
│   │   └── transforms.c       # Clarke/Park transforms
│   ├── hal/                   # Hardware Abstraction Layer
│   │   ├── adc_interface.h
│   │   ├── pwm_interface.h
│   │   └── stm32/             # STM32 implementations
│   └── drivers/               # Low-level drivers
├── test/
│   ├── test_pid.c
│   ├── test_foc.c
│   └── test_transforms.c
├── mocks/                     # Mock hardware interfaces
└── project.yml                # Ceedling configuration
```

## Setting Up Your Environment
Let me create a comprehensive setup guide for you:
Step 1: Install Tools (Windows/Linux/Mac)
```console
sudo apt install gcc-arm-none-eabi gdb-multiarch openocd

# Ruby (for Ceedling)
sudo apt install ruby

# Ceedling
gem install ceedling

# CMake
sudo apt install cmake

```

Step 2: Test Ceedling Installation
```console
ceedling new motor_control_project
cd motor_control_project
ceedling test:all
```