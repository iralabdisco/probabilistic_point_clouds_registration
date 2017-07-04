import csv, sys, os
file_name = sys.argv[1]

with open(file_name, 'rb') as csv_file:
    file_reader = csv.reader(csv_file, delimiter = " ")
    for row in file_reader:
        if row[0] == 'VIEWPOINT':
            trans = row[1:4]
            rot = row[5:8]
            rot.append(row[4])
            print rot
            print trans
            break

cmd = 'pcl_transform_point_cloud '
cmd = cmd + file_name + ' ' + file_name[:-4] + '_gtruth.pcd '
cmd = cmd + '-trans ' + trans[0] + ',' + trans[1]+ ',' + trans[2] + ' '
cmd = cmd + '-quat ' + rot[0] + ',' + rot [1] + ',' + rot[2] + ',' + rot[3]
print cmd
os.system(cmd)
os.system('sed -e 9d '+file_name+' > tmp.pcd')
os.system('cat tmp.pcd > '+file_name)
os.system('rm tmp.pcd')