# wifiscanner

This will publish data from `iwlist scanning` on a topic (published every 5 seconds, hard-coded at the moment). Simply run as `rosrun wifiscanner wifiscanner.py`.

Parameters:

* `~iface`: setting the interface to scan, defaul is `wlan0`
* `~ssid`: defines the ESSID to scan for, default `STRANDS`

For this to work, one needs to be able to run this command: `sudo iwlist %s scanning`, so run `sudo visudo` and edit the sudoers:

```
%sudo ALL=NOPASSWD: /sbin/iwlist
```
