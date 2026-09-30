# GTKterm
GTKTerm is a graphical terminal emulator for Linux and other POSIX-compliant operating systems that allows users to communicate with devices that have a serial interface. Some examples of devices that can be communicated with using GTKTerm include microcontrollers, embedded computers, modems, GPS receivers, and CNC machines.

## installation
```bash
sudo apt install gtkterm
```
## Use 
- connect kv260 and host pc using usb cable
- Open the gtkterm : use GUI or terminal 
    ```bash
    #using terminal
    sudo gtkterm
    ``` 
- configure desired USB port and other settings(Multiple consecutive UART ports will be enumerated automatically by the application).
    ```
    configuration >> port >>
    ```
    here i`m using the configuration listed below
    ```
    port : /dev/ttyUSB1
    Baud Rate: 115200
    Data Bit: 8
    Stop Bit: 1
    No Parity
    ```
    if there is a permission error(ie, can`t open the port) do needfull
```bash
Note: If we start the terminal application before powering up the board, we can see the boot logs (just to know that the board is working as expected).
```