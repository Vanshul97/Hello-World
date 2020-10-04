#!/usr/bin/python3
import jetson.inference
import jetson.utils

import argparse
import sys

import rospy

from nsra_odrive_interface.msg import obj_det

rospy.init_node('bd_object_detection')
bd = rospy.Publisher('bottle_detection', obj_det, queue_size=10)

# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, epilog=jetson.inference.detectNet.Usage() +
                                 jetson.utils.videoSource.Usage() + jetson.utils.videoOutput.Usage() + jetson.utils.logUsage())

parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

is_headless = ["--headless"] if sys.argv[0].find('console.py') != -1 else [""]

try:
	opt = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)

net = jetson.inference.detectNet(opt.network, sys.argv, opt.threshold)

input = jetson.utils.videoSource(opt.input_URI, argv=sys.argv)
output = jetson.utils.videoOutput(opt.output_URI, argv=sys.argv+is_headless)

while True:
	img = input.Capture()
	detections = net.Detect(img, overlay=opt.overlay)
			
	msg = obj_det()
	for detection in detections:
		msg.Center_x.append(detection.Center[0])
		msg.Center_y.append(detection.Center[1])
		msg.Top.append(detection.Top)
		msg.Right.append(detection.Right)
		msg.Left.append(detection.Left)
		msg.Bottom.append(detection.Bottom)
	
	bd.publish(msg)

	output.Render(img)

	output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

	#net.PrintProfilerTimes()

	if not input.IsStreaming() or not output.IsStreaming():
		break

	#test