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
import re

#rostopics = ["/current_pose", "/current_velocity", "/ndt_stat"]
#rostopics = ["/particle_filter/PSM1/state_joint_corrections"] #joint
rostopics = ["/particle_filter/PSM1/position_cartesian_current"] #position
fusetopics = ["/fusion_pose"]
trigger = 0  # This stores the index of the trigger topic. Here, /polaris_h_rad
#selected timestamp
#selected=[str(1598500093867488530), str(1598500095155678200), str(1598500096388493408), str(1598500097641817795), str(1598500098838041776), str(1598500102055087251), str(1598500105315667958), str(1598500108461099249),str(1598500111732114772),str(1598500115052005658),str(1598500118790136529),str(1598500119812887684),str(1598500121982243107),str(1598500124217903258),str(1598500126395322899),str(1598500128632170084)]
#selected=[str(1598500093867488530), str(1598500095664464208), str(1598500097641817795), str(1598500099549660219), str(1598500100131059387), str(1598500103560283707), str(1598500105315667958), str(1598500107498245282),str(1598500109468471202),str(1598500111229883137),str(1598500116807917946),str(1598500118790136529),str(1598500122938637809),str(1598500124697449040),str(1598500130467085244),str(1598500132441747689),str(1598500134389602349),str(1598500136058925711),str(1598500138017456146),str(1598500140007986570),str(1598500141982129938),str(1598500145952272411),str(1598500149916079078),str(1598500153652506279)]

def find_closest(alist, target):
    return min(alist, key=lambda x:abs(int(x)-int(target)))

def list_matching(list1, list2):
    list1_copy = list1[:]
    pairs = []
    for i, e in enumerate(list2):
        if len(list1_copy) != 0:
            elem = str(find_closest(list1_copy, e))
            pairs.append(list1.index(elem))
            list1_copy.remove(elem)
    return pairs

def main():
    # f = open("./combined_results/timestamp.txt")
    # iter_f = iter(f)
    # timestamp = []
    # for line in iter_f:
    #     digit = re.findall(r"\d+\.?\d*",line)
    #     if len(digit) > 0:
    #         timestamp.append(digit[0][0:10] + digit[0][11:])
    # timestamp=timestamp[1:]
    tool_selected = []
    fusion_selected = []
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
                tool_selected.append(str(msg[2]))
    print(len(tool_selected))
    for filename in os.listdir(sys.argv[1]):
        print("Opening", filename)
    bag = rosbag.Bag(os.path.join(sys.argv[1], filename))
    bag_info = yaml.load(bag._get_yaml_info())
    print("Found bag with duration", bag_info['duration'], "seconds")
    print("Parsing", filename)
    flag = 0
    row = []
    index = 0
    for msg in bag.read_messages(topics=fusetopics):
        index += 1
        if msg[0] == fusetopics[trigger]:
            # Trigger found. Get ready to accept other topics
            flag = 1
            row = []
            fusion_selected.append(str(msg[2]))
    print(len(fusion_selected))

    pair = list_matching(tool_selected, fusion_selected)

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
                #if str(msg[2]) >= str(1598661253185376395) and str(msg[2]) <= str(1598661284390631625):
                if index in pair:
                    csv_file_pose = './data_1013/selected_position/' + str(msg[2]) + '.csv'
                    csvfile = open(csv_file_pose, 'wb')
                    writer = csv.writer(csvfile, delimiter=',')
                    row.append(msg[1].pose)
                    writer.writerow(row)
                    
if __name__ == '__main__':
    main()