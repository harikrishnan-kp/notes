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