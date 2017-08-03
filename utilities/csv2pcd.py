#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Convert .csv file from "Challenging data sets for point cloud registration algorithms" datasets from ASL to .pcd
#Check http://projects.asl.ethz.ch/datasets for the data

import csv, sys

points = []
input_file_name = sys.argv[1]

with open(input_file_name, 'r') as csvfile:
    csv_reader = csv.reader(csvfile, delimiter=',')
    header = True;
    for row in csv_reader:
        if header:
            header = False
        else:
            points.append([float(row[0]),float(row[1]),float(row[2])])
out_file_name = input_file_name[:-3]+"pcd"
out_file = open(out_file_name, "w")
width = "WIDTH"
header = ("VERSION .7\n"
          "FIELDS x y z\n"
          "SIZE 4 4 4\n"
          "TYPE F F F\n"
          "COUNT 1 1 1\n" +
          "WIDTH " + str(len(points)) + "\n" +
          "HEIGHT 1\n"
          "VIEWPOINT 0 0 0 1 0 0 0\n"
          "POINTS " + str(len(points)) + "\n" +
          "DATA ascii\n")

out_file.write(header)

for point in points:
    for value in point:
        out_file.write(str(value) + " ")
    out_file.write("\n")

out_file.close()
