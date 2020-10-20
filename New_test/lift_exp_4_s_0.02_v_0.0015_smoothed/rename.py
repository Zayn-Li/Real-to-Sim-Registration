import os
path="./test_pc/"      

f=os.listdir(path)
f.sort();
n=0
for i in f:
    oldname=path+f[n]
    
    newname=path+str(n)+'.ply'

    os.rename(oldname,newname)
    
    n+=1
