import glob, sys, os
from multiprocessing import Pool


problem_folder = sys.argv[1]
commands = []
for file in glob.glob(f"{problem_folder}/*_local.txt"):
    folder = os.path.basename(file)
    folder = folder.replace("_local.txt",'')
    if folder == 'planetary_map':
        folder = 'p2at_met'
    command = f"python /home/simone/Documenti/probabilistic_point_clouds_registration/tester.py {file} /home/simone/Documenti/point_clouds_registration_benchmark/{folder}"
    print(command)
    commands.append(command)
pool = Pool(3)
pool.map(os.system, commands)
