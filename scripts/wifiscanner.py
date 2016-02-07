#!/usr/bin/env python
# collect rssi scan samples from nl80211 (linux >3) supported
# wifi cards
#
# make sure to set the CAP_NET_ADMIN flag if not running as root:
#  setcap cap_net_admin=eip wifi-collect.py
# and to the python interpreter
#  setcap cap_net_admin=eip /usr/bin/python2.7

#from net_tools import all_interfaces, if_nametoindex
from select import select
from subprocess import Popen, PIPE
from sys import exit
import shlex
import re
#import rospy

if __name__ == '__main__':
#    pub = rospy.Publisher('rssi', String)
#    rospy.init_node('rssi_wifi_tap')

    # init
    interfaces = ["en1"]

    if len(interfaces) == 0:
#        rospy.logerr("no wifi interfaces found, are they 'up'?")
        exit(-1)

#    rospy.loginfo("collecting rssi on %s",str(interfaces))

    # communication with tcpdump
    bcn_pattern = '.* (\d+) MHz.* (-\d+)dB signal .* BSSID:([0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]:[0-9A-Fa-f][0-9A-Fa-f]).* Beacon \((\S*)\) .*'
    rts_pattern = '.* (\d+) MHz.* (-\d+)dB .* TA:(%s).*'
    cmdlines = [shlex.split("/usr/sbin/tcpdump -eIi %s" % (iface))
                for iface in interfaces if iface != "wlan2"]
    popens = [Popen(cmd, bufsize=8192,
                    shell=False, stdout=PIPE) for cmd in cmdlines]
    channels = [p.stdout for p in popens]
    interfaces = dict([(f.fileno(),if_name) for (f,if_name) in zip(channels,interfaces)])
    popens     = dict([(f.fileno(),p) for (f,p) in zip(channels,popens)])
    channels   = dict([(f.fileno(),f) for f in channels])

    # we also keep a set of seen beacons and add them to the list to also
    # capture their RTS packets, further increasing the sampling rate
    parser = re.compile(bcn_pattern)
    bssids = {}

    while True:
        r, w, x = select(channels.keys(), [], [], .5)

        for fid in r:
            f, dev = channels[fid], interfaces[fid]
            line = f.readline()
            #print line
            m = parser.match(line)
            if m is not None:
                (freq, rssi, bssid, ssid) = m.groups()[:4]
                if ssid == 'STRANDS':
                    bssids[bssid]=[freq, rssi]
                    print dev, freq, rssi, bssid, ssid
                    print bssids
                #bssid = m.groups()[2]
                #if not bssid in bssids:
                #    bssids.add(bssid)
                #    parser = re.compile("|".join([bcn_pattern]+[rts_pattern%bssid for bssid in bssids]))

            else: # check if the tcpdump process died
                if popens[fid].poll() is not None:
                    rospy.logerr("shutting down interface %s"%(dev))
                    del channels[fid],interfaces[fid]

                    if len(interfaces)==0:
                        rospy.logerr("no more devices to scan on, shutting down!")
                        exit(-1)

