import glob, sys, os
from multiprocessing import Pool


folder = sys.argv[1]
params = ' '.join(sys.argv[2:])
executable = "/home/probabilistic_point_clouds_registration/build/tester"
problem_folder = f'{folder}/devel/registration_pairs'

commands = [f'{executable} {problem_folder}/apartment_local.txt {folder}/eth/apartment -s 0.2 -t 0.2 {params}',
f'{executable} {problem_folder}/stairs_local.txt {folder}/eth/stairs -s 0.2 -t 0.2 {params}',
f'{executable} {problem_folder}/plain_local.txt {folder}/eth/plain -s 0.2 -t 0.2 {params}',
f'{executable} {problem_folder}/hauptgebaude_local.txt {folder}/eth/hauptgebaude -s 0.2 -t 0.2 {params}',
f'{executable} {problem_folder}/gazebo_summer_local.txt {folder}/eth/gazebo_summer -s 0.2 -t 0.2 {params}',
f'{executable} {problem_folder}/gazebo_winter_local.txt {folder}/eth/gazebo_winter -s 0.2 -t 0.2 {params}',
f'{executable} {problem_folder}/wood_summer_local.txt {folder}/eth/wood_summer -s 0.2 -t 0.2 {params}',
f'{executable} {problem_folder}/wood_autumn_local.txt {folder}/eth/wood_autumn -s 0.2 -t 0.2 {params}',
f'{executable} {problem_folder}/p2at_met_local.txt {folder}/planetary/p2at_met -s 0.2 -t 0.2 {params}',
f'{executable} {problem_folder}/box_met_local.txt {folder}/planetary/box_met -s 0.2 -t 0.2 {params}',
f'{executable} {problem_folder}/planetary_map_local.txt {folder}/planetary/p2at_met -s 0.2 -t 0.2 {params}',
f'{executable} {problem_folder}/long_office_household_local.txt {folder}/tum/long_office_household -s 0.1 -t 0.1 {params}',
f'{executable} {problem_folder}/pioneer_slam_local.txt {folder}/tum/pioneer_slam -s 0.1 -t 0.1 {params}',
f'{executable} {problem_folder}/pioneer_slam3_local.txt {folder}/tum/pioneer_slam3 -s 0.1 -t 0.1 {params}',
f'{executable} {problem_folder}/urban05_local.txt {folder}/kaist/urban05 -s 0.2 -t 0.2 {params}']

print('\n'.join(commands))
pool = Pool(3)
pool.map(os.system, commands)
