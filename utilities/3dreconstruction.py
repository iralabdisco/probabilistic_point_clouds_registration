import os
from os.path import isfile

files = []
for f in os.listdir("."): 
    if isfile(f):
        if "_gtruth" not in f and "guess" not in f and ".pcd" in f:
            files.append(f)
files.sort()

for cloud in files:
    #initial guess
    cmd = "../../build-Release/pso_initial_guess "
    cmd = cmd + cloud + " output.pcd -p 100 -s 1 -t 1 -e 1000 -g " + cloud[:-4] + "_gtruth.pcd"
    print cmd
    os.system(cmd)

    cmd = "../../build-Release/point_cloud_registration guess_"
    cmd = cmd + cloud + " output.pcd -m 10 -r 4 -v -s 1 -t 1 -g "
    cmd = cmd + cloud[:-4] + '_gtruth.pcd '
    print cmd
    os.system(cmd)

    cmd = "pcl_concatenate_points_pcd output.pcd aligned_guess_"+cloud
    print cmd
    os.system(cmd)