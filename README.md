# Bluetooth Testing Program for the MSP 432
### MSP_432_Bluetooth_Test

This program is used to test bluetooth communication with the MSP 432. A terminal emulator is used on a phone paired with a bluetooth module to send commands over bluetooth. The MSP 432 then receives the commands and performs an action if the command is recognized.

Recognized commands:
 - STA : Transmit status of system with date and time, arm/disarmed status, and lock/unlocked status
 - LCK : Lock/unlock the door and transmit the outcome
 - ARM : Arm/disarm the alarm and transmit the outcome

 
Links: 
[Bluetooth Module Used] (https://www.amazon.com/gp/product/B00L08GA4Q/ref=oh_aui_detailpage_o00_s00?ie=UTF8&psc=1)
		- Any HC-05 or HC-06 module should work, but the baud rate may need to be adjusted based on the module's specifications. Some modules may also require commands to end with /r or /n and this code does not account for that.
[Bluetooth Terminal App Used] (https://play.google.com/store/apps/details?id=project.bluetoothterminal&hl=en)
		- A lot of other bluetooth terminals are also out there that will work, just keep in mind if they send /r or /n at the end of each transmission.
		