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
rostopics = ["/point_cloud"]
trigger = 0  # This stores the index of the trigger topic. Here, /polaris_h_rad

print(chr(27) + "[2J")
print("Entering", sys.argv[1])

for filename in os.listdir(sys.argv[1]):
    print("Opening", filename)
    bag = rosbag.Bag(os.path.join(sys.argv[1], filename))
    bag_info = yaml.load(bag._get_yaml_info())
    print("Found bag with duration", bag_info['duration'], "seconds")
    print("Parsing", filename)
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
        print(str(msg[2]))
        print(index)
        if flag:
            if msg[0] in rostopics:
		    csv_file_pointcloud = './point_cloud/' + str(msg[2]) + ".csv"
		    csvfile = open(csv_file_pointcloud, 'wb')
		    writer = csv.writer(csvfile, delimiter=',')
		    #		print("0:",msg[0])
		    #		print("1:",msg[1])
		    #		print(msg[1].pose)
		    #		row.append(msg[1].pose)
		    #           print("2:",msg[2])
		    #		print(row)
		    #print("Rad: ", msg[1].multi_array.layout.dim[0].size)
		    row.append(msg[1])
		    writer.writerow(row)
print("Parsed data. Saved as", csv_file_pose)
