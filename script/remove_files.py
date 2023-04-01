import os

directory = "/home/tadeas/my_workspace/src/fusion_radiation/data/"
strings_to_check = ["output_size_avg_best=,1"]

for filename in os.listdir(directory):
    if not filename.startswith("estim_"):
        continue
    filepath = os.path.join(directory, filename)
    if os.path.isfile(filepath):
            with open(filepath, "r") as f:
                first_line = f.readline().strip()
                if not any(s in first_line for s in strings_to_check):
                    os.remove(filepath)
               