# GTKterm
GTKTerm is a graphical terminal emulator for Linux and other POSIX-compliant operating systems that allows users to communicate with devices that have a serial interface. Some examples of devices that can be communicated with using GTKTerm include microcontrollers, embedded computers, modems, GPS receivers, and CNC machines.

## installation
```bash
sudo apt install gtkterm
```
## use 

Open the gtkterm 
```bash
sudo gtkterm
``` 
Multiple consecutive UART ports will be enumerated. Connect the serial terminal application to the right USB port (its /dev/ttyUSB1) and use the parameters listed below.
```
Baud Rate: 115200
Data Bit: 8
Stop Bit: 1
No Parity
```

```
Note: If we start the terminal application before powering up the board, we can see the boot logs (just to know that the board is working as expected).
```