# ledCAN
A multipurpose buck converter with CAN interface.
![image](./kicad/pcb/ledCAN.png)

## Clone the project
This project work with M0A21BSP submodule from Nuvoton corporation. You need to initialize submodules by passing ``--recurse-submodules`` to the ``git clone`` command.
```
git clone https://github.com/MIH-pi/ledCAN.git --recurse-submodules
```

## Prepare environment
1. Installing tools
```
sudo apt install kicad build-essential gcc-arm-none-eabi libtool libusb-1.0-0-dev
```
2. Build customize version of OpenOCD for Nuvoton devices follow by the instructions
```
git clone https://github.com/OpenNuvoton/OpenOCD-Nuvoton.git
cd OpenOCD-Nuvoton
./bootstrap
./configure --enable-nulink
make
sudo make install
```

## Credits
We are thankful that [Nuvoton corporation](https://www.nuvoton.com) (MCU),[PANJIT](https://www.panjit.com.tw) (MOSFET), [Texas Instruments](https://ti.com) (CAN SBC) provid samples and technical support.
