import os

path="./clusters0.0010.txt"
offset = []
idx = []

with open(path,'r') as f:

    for line in f.readlines():

        if line.startswith('CLUSTEROFFSETS'):

            read_offset = 1
            continue
        
        elif line.startswith('CLUSTEINDICES'):
            
            read_offset = 0
            continue
        
        if read_offset:
            offset.append(int(line))
        else :
            idx.append(int(line))

with open('reorganized_cluster.txt', 'w') as fw:

    start_id = 0
    
    for end_id in offset:
        
        fw.write(str(idx[start_id : end_id])+"\n")
        start_id = end_id
        

