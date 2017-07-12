import os, sys

def frange(start, end, step):
    tmp = start
    while(tmp < end):
        yield tmp
        tmp += step  

source = sys.argv[1]
target = sys.argv[2]

for filt in frange(0.1,10,0.1):
    cmd = "../../build-Release/point_cloud_registration "
    cmd = cmd + source + " " + target
    cmd = cmd + " -m 15 -r 5 -s " + str(filt) + " -t " + str(filt) + " -g "
    cmd = cmd + source[:-4]+ '_gtruth.pcd'
    print cmd 
    os.system(cmd)

