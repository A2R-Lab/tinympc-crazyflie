To build the firmware for the tinympc controller on Crazyflie using WSL on Windows, follow these organized steps:

### 1. Install WSL on Windows
- Open Command Prompt as Administrator.
- Run: `wsl --install`

### 2. Set Up Ubuntu 20.04 in WSL
- Restart your computer and select the newly installed Ubuntu 20.04.

### 3. Update and Install Build Dependencies
- Update the package list:
  ```bash
  sudo apt update && sudo apt upgrade -y
  ```
- Install necessary build tools and dependencies:
  ```bash
  sudo apt install -y build-essential cmake git python3 python3-dev
  ```

### 4. Install Python and CFclient on Windows
- Download and install Python 3.7 or higher from [Python.org](https://www.python.org/).
- Check the option to add Python to PATH during installation.
- Open PowerShell or Command Prompt and run:
  ```bash
  pip install cfclient
  ```

### 5. Clone the Firmware Repository
- In WSL, navigate to your desired directory and clone the repository:
  ```bash
  git clone --recursive https://github.com/ginoekzhang/a2r-tinympc-crazyflie-firmware.git
  cd a2r-tinympc-crazyflie-firmware
  ```

### 6. Configure the Build Settings
- Run the configuration menu:
  ```bash
  make menuconfig
  ```
- Navigate to:
  - **App Layer Configuration**: Enable "App entry point."
  - **Controllers and Estimators**: Enable "Out of tree."
- Save the configuration as `.config`.

### 7. Build the Firmware
- Compile the firmware using all available CPU cores:
  ```bash
  make -j$(nproc)
  ```

### Additional Notes:
- After building, use CFclient to flash the firmware onto your Crazyflie.
- Verify Python and CFclient installations by running `python --version` and `cfclient --version`.

This guide should help you set up the environment and build the firmware successfully. If issues arise, refer to detailed Crazyflie documentation or community resources for
further assistance.
