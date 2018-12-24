## STM32_NUCLEO-F401RE_IHM01A1_ChibiOS
Port of the X-NUCLEO-IHM01A1 Board Support Package and motorcontrol drivers to ChibiOS.

**This software is in some places untested!**

### ToDo
* Test 2nd and 3rd device
* Implement separate init structs for each device
* Implement separate flag interrupt and error handlers for each device
* Implement further commands
* Fix reset command
* ...

### Getting started
1. Build using `make`
2. Upload binary to your ST Nucleo Board
3. Open a serial console (38400, 8n1)
4. Type in `help` to get a list of supported commands
5. Type in any command to get further instructions
