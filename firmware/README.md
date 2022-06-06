# Firmware of the ledCAN module
``M0A21BSP`` BSP submodule from Nuvoton corporation. <br>
``HelloWorld`` Project template for M0A21 serise. <br>
``ISP_CAN`` This is a loader firmware implement ISP function via CAN interface. <br>
``SyncBuck`` This is a buck converter test code with PID controller. <br>
``NeoPixel`` This is a NeoPixel strandtest with a modified version of ``Adafruit_NeoPixel`` library. <br>
``CAN_TxRx`` This is a CAN test code for mode and color select. <br>
``FlashConfig`` This is a code for config bits setup and accessing data flash. <br>
``ledCAN`` The full function firmware for final product. <br>
``id`` Generate CAN ID bianry file from shell script. <br>

## Usage
1. Build a project. 
```
# The output binary file will be in the Objects folder
cd to project directory
make clean && make
```
2. Run OpenOCD-Nuvoton and connect via telenet
```
sudo openocd -f /usr/local/share/openocd/scripts/interface/nulink.cfg -f /usr/local/share/openocd/scripts/target/numicroM0.cfg
telnet localhost 4444
```
2. Erase the whole chip
```
numicro chip_erase
```
3. To program the LDROM
```
flash write_image erase ISP_CAN.bin 0x00100000
```
4. To program the APROM
```
flash write_image erase ledCAN.bin 0x00000000
```
5. To program the config bits
```
# Config 0: 0xFDFFF97E, I/O pull-up, extern POR time, disable reset pin, boot from LDROM, enable data flash
numicro write_isp 0x00300000 0xFDFFF97E
# Config 1: 0x00007E00, DFBA=0x07E00 (data flash base address)
numicro write_isp 0x00300004 0x00007E00
```
6. To program the data flash
```
# numicro write_isp DFBA CAN_ID
numicro write_isp 0x00007E00 0x201
```
