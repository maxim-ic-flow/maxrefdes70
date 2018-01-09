# MAXREFDES70
Firmware for the MAXREFDES70 water flow meter reference design

## Official Product Page

https://www.maximintegrated.com/en/design/reference-design-center/system-board/5969.html

## Tools

The MAXREFDES70 firmware can be built using Silab's Simplicity Studio, however, the firmware requires a specific version of the GNU ARM toolchain which must be manually installed and configured within Simplicity Studio.

Simplicity Studio:  https://www.silabs.com/products/development-tools/software/simplicity-studio

GNU ARM 4.8-2013-q4:  https://launchpad.net/gcc-arm-embedded/4.8/4.8-2013-q4-major

## Tool Configuration

In Simplicity Studio, add the 4.8 toolchain.  Select "Window", then "Preferences" from the main menu.  Next, type in "Toolchains" in the search box. Select the "Add" button and supply a path or browse to the location of the 4.8 toolchain.

![alt text](https://github.com/maxim-ic-flow/maxrefdes70/blob/master/docs/readme_images/toolchain_image.jpg "Toolchain")

Project Explorer should look like this:

![alt text](https://github.com/maxim-ic-flow/maxrefdes70/blob/master/docs/readme_images/project_image.jpg "Project Explorer")

## Attaching a Debugger

J1 can be used to attach a debugger to the MAXREFDES70. A custom adapter or jumper wires can be used to connect the board to an ARM Cortex debugger.

![alt text](https://github.com/maxim-ic-flow/maxrefdes70/blob/master/docs/readme_images/debugger_image.jpg "Typical debugger setup")

The pinout for J1 is:
![alt text](https://github.com/maxim-ic-flow/maxrefdes70/blob/master/docs/readme_images/board_debug_pinout.jpg "Typical debugger setup")

The pinout for a standard 20-pin ARM debugger is:
![alt text](https://github.com/maxim-ic-flow/maxrefdes70/blob/master/docs/readme_images/20pin.png "20-PIN JTAG/SW Interface")

| ARM 20-PIN CONNECTOR | REFDES70 6-PIN CONNECTOR |
|----------------------|--------------------------|
| VCC(1)               | VBAT(J1-6)               |
| GND(4)               | GND(J1-4)                |
| SWDIO(7)             | DBG_SWDIO(J1-1)          |
| SWCLK(9)             | DBG_SWCLK(J1-3)          |
| RESET(15)            | RESET(J1-2)              |

## Notes

This firmware uses almost the entire 32K flash in the EFM32ZG110F32 MCU.  Newer toolchains push the code size just beyond what will fit but they can be used if space is freed up.

