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
  git clone --recursive https://github.com/ginoekzhang/a2r-tinympc-crazyflie-firmware.git
  cd a2r-tinympc-crazyflie-firmware
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
