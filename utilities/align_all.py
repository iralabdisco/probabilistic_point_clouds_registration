import os
from os.path import isfile

files = []
for f in os.listdir("."): 
    if isfile(f):
        if "_gtruth" not in f and "guess" not in f:
            files.append(f)
files.sort()

targets = files[:-1]
sources = files[1:]
for source, target in zip(sources, targets):
    cmd = "../../build-Release/pso_initial_guess "
    cmd = cmd + source + " " + target[:-4] + '_gtruth.pcd '
    cmd = cmd + " -p 50 -s 1 -t 1 -e 300 -g "
    cmd = cmd + source[:-4]+ '_gtruth.pcd'
    os.system(cmd)