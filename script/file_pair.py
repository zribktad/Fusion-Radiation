import os

def find_file_pairs(directory,keyword = ""):
    file_pairs = []
    dict_list = os.listdir(directory)
    dict_list=sorted(dict_list)
    for filename in dict_list:
        if filename.startswith('rad_src_') and keyword in filename:
            rad_src_file = filename
            estim_file = 'estim_' + filename[8:]
            if os.path.isfile(os.path.join(directory, estim_file)):
                file_pairs.append((rad_src_file, estim_file))
    return file_pairs
