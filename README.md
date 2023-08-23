# STM32 USB-CDC-VCP demonstration project
This project implements USB CDC Virtual COM Port functionality to STM32F10x devices with USB FS device peripheral.

It is for educational purpose and is not meant to be used in production environment.

# Features
Codes are made straightforward: being small, fast and easy to understand.

Easy to use: only 4 functions in user interface.
  * `USB_VCP_init`: initializes USB peripheral registers.
  * `USB_VCP_available`: check length of data received from host.
  * `USB_VCP_readBytes`: get data from host.
  * `USB_VCP_writeBytes`: send data to host.
