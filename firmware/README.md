# Firmware of the ledCAN module
``M0A21BSP`` BSP submodule from Nuvoton corporation. <br>
``HelloWorld`` Project template for M0A21 serise. <br>
``ISP_CAN`` This is a loader firmware implement ISP function via CAN interface. <br>
``SyncBuck`` This is a buck converter test code with PID controller. <br>
``NeoPixel`` This is a NeoPixel strandtest with a modified version of ``Adafruit_NeoPixel`` library. <br>

## Usage
1. Build
Output binary file will be in the ``Objects`` folder after build. 
```
cd to project directory
make clean && make
```
2. Erase whole chip and verify each block
```
numicro chip_erase
flash erase_check NuMicro.flash_config
flash erase_check NuMicro.flash_ldrom
flash erase_check NuMicro.flash_aprom
```
3. To flash the config bits (not complet yet)
```
# Create configure binary file
echo -n -e "\x7F\xFB\xFF\xFD\xFF\xFF\xFF\xFF\x5A\xFF\xFF\xFF" >> config.bin
```
4. To flash the LDROM
```
openocd -f ../scripts/interface/nulink.cfg -f ../scripts/target/numicroM0.cfg -c "init;reset init;flash erase_address 0x00100000 0x800;flash erase_check NuMicro.flash_ldrom;flash write_bank NuMicro.flash_ldrom ISP_CAN.bin 0;flash verify_bank NuMicro.flash_ldrom ISP_CAN.bin 0;exit;"
```
5. To flash the APROM
```
openocd -f ../scripts/interface/nulink.cfg -f ../scripts/target/numicroM0.cfg -c "init;reset init;flash erase_address 0x00000000 0x8000;flash erase_check NuMicro.flash_aprom;flash write_bank NuMicro.flash_aprom HelloWorld.bin 0;flash verify_bank NuMicro.flash_aprom HelloWorld.bin 0;exit;"
```
