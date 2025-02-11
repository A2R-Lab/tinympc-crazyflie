To build the firmware for the tinympc controller on Crazyflie using WSL on Windows, follow these organized steps:
# Prerequisites for Windows
### Windows: Install WSL, Ubuntu, and CFclient
- Open Command Prompt as Administrator.
- Run: `wsl --install`
- Download and install Python 3.7 or higher from [Python.org](https://www.python.org/).
- Check the option to add Python to PATH during installation.
- Open PowerShell or Command Prompt and run:
  ```bash
  pip install cfclient
  ```
# Steps for Ubuntu Linux  / Windows (WSL)
### 1. Update and Install Build Dependencies
- Update the package list:
  ```bash
  sudo apt update && sudo apt upgrade -y
  ```
- Install necessary build tools and dependencies:
  ```bash
  sudo apt install -y build-essential cmake git python3 python3-dev
  ```

### 2. Clone the Firmware Repository
- In WSL, navigate to your desired directory and clone the repository:
  ```bash
  GIT_CLONE_PROTECTION_ACTIVE=false git clone --recursive git@github.com:A2R-Lab/tinympc-crazyflie.git
  cd tinympc-crazyflie
  ```

### 3. Configure the Build Settings
- Run the configuration menu:
  ```bash
  make menuconfig
  ```
- Navigate to:
  - **App Layer Configuration**: Enable "App entry point."
  - **Controllers and Estimators**: Enable "Out of tree."
- Save the configuration as `.config`.

### 4. Build the Firmware
- Compile the firmware using all available CPU cores:
  ```bash
  make -j$(nproc)
  ```



# For MacOS

## Prerequisites

- **Homebrew**: Ensure you have Homebrew installed on your macOS.
- **SWIG**: Use Homebrew to install SWIG.

  ```bash
  brew install swig
  ```

## Installation Steps

### Step 1: Clear Build

1. **Clean the Build**: This will erase the current configuration.

   ```bash
   make clean
   ```

2. **Manually Remove Files (Optional)**:

   - Remove the `build` directory:

     ```bash
     rm -rf build
     ```

   - Navigate to the `src` directory and remove object files:

     ```bash
     cd src
     rm *.o
     ```

### Step 2: Write the `.config` File

1. **Generate Configuration**: Use `make nconfig` to configure the project.

   ```bash
   make nconfig
   ```

2. **Configuration Settings**:

   - Navigate to **App layer config**:
     - Set **Entry point** to **on**.

   - Navigate to **Controllers**:
     - Set **Out of tree controller** to **on**.

### Step 3: Build the Project

- Run the following command to build the project:

  ```bash
  make
  ```

  # Flashing the CrazyFlie
  - Run the client with command: cfclient
  - Run the following with the CrazyRadio plugged in, and a CrazyFlie turned on and near you:

    ```bash
    CLOAD_CMDS="-w radio://0/80/2M/E7E7E7E7E7" make cload
    ```

  - Manual option - put CrazyFlie in bootloader mode by holding down power button (3 seconds)
  - Full instructions [here](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/#using-crazyradio).

  # Docker Setup and Usage
  To use the docker container, clone the repo and run the following commands from the root - 

 - ```docker build -t crazyflie-build . ```
 - ```docker run -it --rm -v "$(pwd):/workspace" crazyflie-build```



