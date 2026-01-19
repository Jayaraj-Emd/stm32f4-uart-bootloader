STM32F4 UART Bootloader
======================

Overview
--------

This project implements a custom UART based bootloader for the STM32F407 microcontroller.

The system consists of three main components:
- Bootloader firmware running at the beginning of flash memory  
- User application firmware located at a fixed offset in flash  
- A Python based host tool for communication and programming over UART  

The bootloader supports a command based protocol with CRC verification to ensure reliable communication and safe flash operations.
It provides functions for reading device information, erasing flash sectors, programming application binaries, configuring flash protection, and jumping to the user application.

This project demonstrates practical bootloader architecture used in embedded production systems, including memory partitioning, vector table relocation, command based firmware update, and controlled application execution.
The entire system is developed using STM32CubeIDE and STM32 HAL drivers.


Key Features
------------

- UART based custom bootloader implementation  
- In-system firmware update without external programmer  
- CRC based packet verification for reliable communication  
- Flash sector erase and memory write operations  
- Read device ID and flash protection status  
- Enable and disable flash sector protection  
- Jump to user application from bootloader  
- Python based host tool for command and firmware transfer  
- Compatible with STM32CubeIDE and HAL drivers  


Project Structure
-----------------

Bootloader/  
|-- Bootloader      -> Bootloader firmware project (STM32CubeIDE)  
|-- Application     -> User application firmware project  
|-- Host            -> Python UART host tool  
|-- README.md  


Target Hardware
---------------

Microcontroller        : STM32F407VG  
Board                  : STM32F4 Discovery (or compatible)  
Development Environment: STM32CubeIDE  
Communication Interface: UART  


Memory Layout
-------------

Bootloader region:  
Start address : 0x08000000  
Size          : 128 KB  

Application region:  
Start address : 0x08020000  
Size          : Remaining flash  


Boot Process
------------

On reset, execution starts from the bootloader at address 0x08000000.

The bootloader checks the user button at startup.

If the button is pressed:  
The bootloader stays in command mode and waits for host commands.

If the button is not pressed:  
The bootloader jumps to the user application.

The jump sequence:
- Load application stack pointer  
- Set MSP  
- Jump to application reset handler  


Supported Bootloader Commands
-----------------------------

BL_GET_VER                 Get bootloader version  
BL_GET_HELP                Get supported command list  
BL_GET_CID                 Get MCU chip ID  
BL_GET_RDP_STATUS          Read flash protection level  
BL_GO_TO_ADDR              Jump to specified address  
BL_FLASH_ERASE             Erase flash sectors  
BL_MEM_WRITE               Write data to flash memory  
BL_EN_RW_PROTECT           Enable flash sector protection  
BL_DIS_R_W_PROTECT         Disable flash protection  
BL_READ_SECTOR_P_STATUS    Read protection status  


Host Tool
---------

A Python based UART host tool is provided in the Host folder.

The tool performs:
- Sending bootloader commands  
- Programming application binary files  
- Receiving acknowledgments and responses  
- Displaying status information  

The tool uses the pyserial library for serial communication.  


Build and Flash Instructions
----------------------------

Bootloader:

1. Open the Bootloader project in STM32CubeIDE  
2. Build the project  
3. Flash the bootloader at address 0x08000000  


Application:

1. Open the Application project in STM32CubeIDE  
2. Verify linker script start address is 0x08020000  
3. Build the project  
4. Generate binary file (.bin)  
5. Program using the host tool  


Running the Host Tool
--------------------

Install Python and pyserial:

pip install pyserial  

Run:

python stm32_uart_bootloader_tool.py  

Enter the COM port and select the required command from the menu.  


Development Notes
-----------------

- Developed using STM32CubeIDE  
- Uses STM32 HAL drivers  
- CRC peripheral used for verification  
- Flash operations use HAL APIs  
- Vector table relocation handled in application  


Future Improvements
-------------------

- Application image authentication  
- Encrypted firmware update support  
- Rollback protection mechanism  
- Dual image firmware update support  
- Application integrity verification before execution  
