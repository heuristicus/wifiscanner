# collect rssi scan samples from nl80211 (linux >3) supported
# wifi cards
#
# make sure to set the CAP_NET_ADMIN flag if not running as root:
#  setcap cap_net_admin=eip wifi-collect.py
# and to the python interpreter
#  setcap cap_net_admin=eip /usr/bin/python2.7

#from net_tools import all_interfaces, if_nametoindex
import rospy
from subprocess import check_output, CalledProcessError
import re

def get_iwconf_data(interface):
    """Get data from the iwconfig command

    :param interface: String of the network interface for which to gather data
    """
    iwconf_pattern = "^.*ESSID:\"(.*)\"  \n.*Frequency:(\d+.\d+ \w+).*Access Point: ([0-9ABCDEF:]+).*\n.*Bit Rate=(\d*\.*\d* [A-Za-z]*\/s).*Tx-.*\n.*\n.*\n.*Link Quality=(\d+\/\d+).*Signal level=(.*)  \n"
    iwconf_cmd = "iwconfig {0}".format(interface).split()
    parser_iwconfig = re.compile(iwconf_pattern, re.MULTILINE)

    try:
        iwconf_out = check_output(iwconf_cmd)
        iwconf_info = {}
        (iwconf_info['essid'],
         iwconf_info['freq'],
         iwconf_info['ap_hwaddr'],
         iwconf_info['bitrate'],
         iwconf_info['link_quality'],
         iwconf_info['signal_level']
        ) = parser_iwconfig.search(iwconf_out).groups()
        return iwconf_info
    except CalledProcessError:
        rospy.logwarn('failed to get iwconfig data')
        return None

def get_iwlist_data(interface, check_essid=None):
    """Get data from the iwlist scanning command on the given interface.

    :param interface: String of the network interface for which to gather a list
    of access points
    :param check_essid: String name of the essid that you are interested in
    looking for. Setting this will cause the function to only return access
    point data for access points with the given ESSID

    """
    # This regex will not get data for any device which does not have an ESSID
    # (I guess those are hidden?)
    iwlist_pattern = '^.*Cell (\d+) - Address: ([0-9ABCDEF:]+).*\n.*Channel:(\d+)\n.*\n.*Quality=(\d+\/\d+).*Signal level=([-0-9]+).*\n.*\n.*ESSID:"(.+)"'
    # To run this command without sudo requiring a password, add
    # ALL ALL=(ALL) NOPASSWD: /sbin/iwlist
    # to sudoers with sudo visudo
    iwlist_cmd = "sudo -S iwlist {0} scanning".format(interface).split()
    parser_iwlist = re.compile(iwlist_pattern, re.MULTILINE)

    try:
        iwlist_out = check_output(iwlist_cmd)
        iwlist_status_list = []
        for match in parser_iwlist.finditer(iwlist_out):
            iwlist_info = {}
            (iwlist_info['cell'],
             iwlist_info['bssid'],
             iwlist_info['channel'],
             iwlist_info['quality'],
             iwlist_info['signal'],
             iwlist_info['essid']) = match.groups()
            # If monitoring a specific essid, only output the data for that essid, and ignore all the others.
            if check_essid:
                if iwlist_info['essid'] == check_essid:
                    iwlist_status_list.append(iwlist_info)
            else:
                iwlist_status_list.append(iwlist_info)
        return iwlist_status_list
    except CalledProcessError:
        rospy.logwarn('failed to get wifi scan')
        return None
