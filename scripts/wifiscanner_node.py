#!/usr/bin/env python

from wifiscanner.utils import get_iwconf_data, get_iwlist_data
import rospy
from pprint import pformat
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class WifiScannerNode(object):

    def __init__(self):
        rospy.init_node('wifiscanner')
        self.iwlist_pub = rospy.Publisher('/wifiscanner/iwlist_output', DiagnosticArray, queue_size=10)
        self.iwconf_pub = rospy.Publisher('/wifiscanner/iwconfig_output', DiagnosticArray, queue_size=5)

        self.sleep_time = rospy.get_param('~sleep_time', 8)

        # init
        self.monitored_essid = rospy.get_param('~monitor_essid', None)
        self.interface = rospy.get_param('~iface', 'wlan0')
        self.interfaces = [self.interface]
        self.scan_ssid = rospy.get_param('~ssid', 'STRANDS')

    def spin(self):
        while not rospy.is_shutdown():
            self.publish_status(self.iwconf_pub, self.populate_status(get_iwconf_data(self.interface), 'wifi conf'))
            self.publish_status(self.iwlist_pub, self.populate_status(get_iwlist_data(self.interface)))
            
            rospy.sleep(self.sleep_time)
                    
    def publish_status(self, pub, status):
        diag = DiagnosticArray()
        diag.header.stamp = rospy.Time.now()
        diag.status = status if isinstance(status, list) else [status]
        pub.publish(diag)
    
    def _populate_status(self, cmd_out, name):
        """Populate status helper function
        """
        status = DiagnosticStatus()
        status.hardware_id = "wifi"
        status.name = name
        status.level = status.OK
        status.message = pformat(cmd_out)

        for k,v in cmd_out.items():
            status.values.append(
                KeyValue(k,str(v)),
            )

        return status

    def populate_status(self, cmd_out, name='wifi scan'):
        if cmd_out:
            if isinstance(cmd_out, list):
                return [self._populate_status(out, name) for out in cmd_out]
            else:
                return self._populate_status(cmd_out, name)
        else:
            # if the command fails, then we get a NoneType return, in which case
            # return an empty array
            return DiagnosticArray()

if __name__ == '__main__':
    scanner_node = WifiScannerNode()
    scanner_node.spin()
