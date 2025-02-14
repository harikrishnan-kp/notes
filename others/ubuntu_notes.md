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