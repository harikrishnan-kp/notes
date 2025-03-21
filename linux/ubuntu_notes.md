# Notes
## system sleep
check sleep timeout on ac power
```bash
gsettings get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout
```
it will return the current sleep time out in seconds, we can
change this timeout using 
```bash
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-timeout <time_in_sec>
```
## scp 
- tool for accessing files from a remote system, if we know the IP address and password
- by default scp is using SFTP protocol to establish connection between remote server
- scp can work both in push and pull mode
```bash
#scp pull
scp <username>@<ip address>:<remote file dir> <dowload path> 

#scp push
scp <dowload path> <username>@<ip address>:<remote file dir> 
```
## error while fetching from git
```bash
error: RPC failed; curl 92 HTTP/2 stream 5 was not closed cleanly: CANCEL (err 8)
error: 6791 bytes of body are still expected
fetch-pack: unexpected disconnect while reading sideband packet
fatal: early EOF
fatal: fetch-pack: invalid index-pack output
```
solution: git config --global http.postBuffer 524288000

## mkdir
- -p (or --parents) option, which allows you to create a parent directory along with multiple subdirectories in a single command
```
mkdir -p /path/to/parent_directory/{subdir1,subdir2,subdir3}
```
## Access the GUI of the raspberry pi system

- Enable the Remote Login in the host device(Raspberry Pi) settings
    - Goto  setting >> system >>Remote Login
    - Set the username and password in this
- Install the remmina in the client desktop
```
	sudo apt install remmina -y
```
- Now open the Remmina by entering the below command from the terminal 
```
remmina
```
- Now set RDP as the protocol
- Enter the server in the format <raspberry_ip>:<port> (port can be seen in the host desktop setting >> Remote Login >> you can see the port) 192.168.3.187:3389
- Enter the username and password
    - Username:- pi4
    - Password:- netrasemi
- Then click save and connect option. 
```bash
Note: When Remmina is opened, instead of specifying the protocol, server, username and password each time when we log in, just click the “plus” button to add a new connection item, then give protocol, server, username and password, save it. Thereafter, we can just open the terminal by opening this item (double click).
```
