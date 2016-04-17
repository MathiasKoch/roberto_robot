#!/usr/bin/env python

# The MIT License (MIT)
#
# Copyright (c) 2015 Richard Hull
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


# Example usage:
#
#   from oled.device import ssd1306, sh1106
#   from oled.render import canvas
#   from PIL import ImageFont, ImageDraw
#
#   font = ImageFont.load_default()
#   device = ssd1306(port=1, address=0x3C)
#
#   with canvas(device) as draw:
#      draw.rectangle((0, 0, device.width, device.height), outline=0, fill=0)
#      draw.text(30, 40, "Hello World", font=font, fill=255)
#
# As soon as the with-block scope level is complete, the graphics primitives
# will be flushed to the device.
#
# Creating a new canvas is effectively 'carte blanche': If you want to retain
# an existing canvas, then make a reference like:
#
#    c = canvas(device)
#    for X in ...:
#        with c as draw:
#            draw.rectangle(...)
#
# As before, as soon as the with block completes, the canvas buffer is flushed
# to the device

from PIL import Image, ImageDraw, ImageFont
import rospy
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt16
from std_msgs.msg import MultiArrayDimension

import socket
import fcntl
import struct

import rospkg


class canvas(object):
    """
    A canvas returns a properly-sized `ImageDraw` object onto which the caller
    can draw upon. As soon as the with-block completes, the resultant image is
    published to the publisher.
    """

    def __init__(self, publisher):
        self.draw = None
        self.image = Image.new('1', (128, 64))
        self.pub = publisher

    def __enter__(self):
        self.draw = ImageDraw.Draw(self.image)
        return self.draw

    def __exit__(self, type, value, traceback):
        if type is None:
            # do the drawing onto the device
            pixels = UInt8MultiArray()
            bits = list(self.image.convert("1").getdata())
            vals = list()
            w,h = self.image.size
            for i in range(0,h/8):
                for idx in reversed(range(0,w)):
                    stop = (i+1)*8*w-1
                    start = (stop+1)-8*w+idx
                    vals.append(int(''.join([str(mli) for mli in bits[start:stop:w]])[::-1],2))
            pixels.layout.dim.append(MultiArrayDimension())
            pixels.layout.dim[0].label = "image"
            pixels.layout.dim[0].size = (w*h)/8
            pixels.layout.dim[0].stride = (w*h)/8
            pixels.layout.data_offset = 0
            pixels.data = vals
            self.pub.publish(pixels)

        del self.draw   # Tidy up the resources
        return False    # Never suppress exceptions




def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])


def callback(data):
    rospack = rospkg.RosPack()
    font2 = ImageFont.truetype(rospack.get_path('roberto_menu') + "/fonts/C&C Red Alert [INET].ttf", 12)
    global pubL
    with canvas(pubL) as draw:
            draw.text((10, 10), "Voltage: " + str(data.data) + " mV", font=font2, fill=1)

pubL = None

def talker():
    rospy.init_node('talker', anonymous=True)
    rospack = rospkg.RosPack()

    font = ImageFont.load_default()
    font2 = ImageFont.truetype(rospack.get_path('roberto_menu') + "/fonts/C&C Red Alert [INET].ttf", 12)
    pubR = rospy.Publisher('displayR', UInt8MultiArray, queue_size=10)
    
    global pubL
    pubL = rospy.Publisher('displayL', UInt8MultiArray, queue_size=10)
    rospy.Subscriber("battery_stats", UInt16, callback)
    rate = rospy.Rate(0.2) # 1hz
    while not rospy.is_shutdown():
        #with canvas(pubL) as draw:
        #    logo = Image.open('images/pi_logo.png')
        #    draw.bitmap((32, 0), logo, fill=1)
        with canvas(pubR) as draw:
            draw.text((10, 10), "IP: " + get_ip_address('wlan2'), font=font2, fill=1)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
