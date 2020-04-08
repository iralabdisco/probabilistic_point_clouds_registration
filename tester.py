import csv, os, sys, subprocess,tqdm

executable = "/home/simone/Documenti/probabilistic_point_clouds_registration/build/probabilistic_point_cloud_registration"
problem_file = sys.argv[1]
folder = sys.argv[2]
# folder = "p2at_met"
results = []
command = []
parameters = ["-r 1000","-s 0.1", "-t 0.1", "-m 5", "-i 40", "-p 0.75"]
_, result_file = os.path.split(problem_file)
result_file = result_file.replace(".txt", "_result.txt")

with open(f"{result_file}", mode="w") as out_file:
    result_writer = csv.writer(
        out_file, delimiter=";", quotechar='"', quoting=csv.QUOTE_MINIMAL
    )
    result_writer.writerow(["#", executable] + parameters)
    result_writer.writerow(["id", "initial_error", "final_error", "transformation"])
    with open(sys.argv[1]) as csvfile:
        file_reader = csv.DictReader(csvfile, delimiter=" ")
        for row in tqdm.tqdm(file_reader):
            source = f"{folder}/{row['source']}"
            target = f"{folder}/{row['target']}"
            transformation = []
            for i in range(1, 13):
                transformation.append(row[f"t{i}"])

            command = [executable, source, target] + parameters + transformation
            # print(command)
            result = subprocess.check_output(command).decode("utf8").split(",")
            # result = [float(x) for x in result]
            result = [row["id"]] + result
            #print(result)
            result_writer.writerow(result)
            out_file.flush()
