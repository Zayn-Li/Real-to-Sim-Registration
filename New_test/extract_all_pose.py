"""
This script collects data from all the bag files in the directory passed as the first
argument when launching. In particular, it collects the rostopics listed in the
concerned list and generates a CSV file containing the same. The readings are triggered
by the trigger topic.
Author: Dhruv Ilesh Shah
shahdhruv@cmu.edu | dhruv.shah@iitb.ac.in
"""

import rosbag
import numpy as np
import sys, os
import csv
import yaml

#rostopics = ["/current_pose", "/current_velocity", "/ndt_stat"]
rostopics = ["/particle_filter/PSM1/position_cartesian_current"]
trigger = 0  # This stores the index of the trigger topic. Here, /polaris_h_rad
print(chr(27) + "[2J")
print("Entering", sys.argv[1])
for filename in os.listdir(sys.argv[1]):
    print("Opening", filename)
    bag = rosbag.Bag(os.path.join(sys.argv[1], filename))
    bag_info = yaml.load(bag._get_yaml_info())
    print("Found bag with duration", bag_info['duration'], "seconds")
    print("Parsing", filename)

    csv_file_pose = filename + "_position_selected.csv"
    csv_file_time = filename + "_time_selected.csv"
    csvfile = open(csv_file_pose, 'wb')
    writer = csv.writer(csvfile, delimiter=',')
    csvfile1 = open(csv_file_time, 'wb')
    writer1 = csv.writer(csvfile1, delimiter=',')
    flag = 0
    row = []

    index = 0
    for msg in bag.read_messages(topics=rostopics):
        index += 1
        if msg[0] == rostopics[trigger]:
            # Trigger found. Get ready to accept other topics
            flag = 1
            row = []
            row1 = []
            # print "- - - - - - - - - - - - - - - - - - - - - - - -"
        if flag:
            if msg[0] in rostopics:
                #		print("0:",msg[0])
                #		print("1:",msg[1])
                #		print(msg[1].pose)
                #		row.append(msg[1].pose)
                #       print("2:",msg[2])
                #		print(row)
                if msg[0] == "/particle_filter/PSM1/position_cartesian_current":
                    #print("Rad: ", msg[1].multi_array.layout.dim[0].size)
			row.append(msg[2])
		        row.append(msg[1].pose)
		        row1.append(msg[2])
		        writer.writerow(row)
		        writer1.writerow(row1)
#                elif msg[0] == "/current_velocity":
#                    print("Odom: ", msg[1].pose.pose.position)
#                    row.extend([msg[1].pose.pose.position.x, msg[
#                               1].pose.pose.position.y, msg[1].pose.pose.position.z])
#                    flag = 0
#                    writer.writerow(row)
print("Parsed data. Saved as", csv_file_pose)
