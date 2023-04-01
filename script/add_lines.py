import os

# Nastavte cestu k priečinku, ktorý chcete prehľadať
folder_path = "/home/tadeas/my_workspace/src/fusion_radiation/data/"

# Prejdite všetky súbory v priečinku
for file_name in os.listdir(folder_path):
    if file_name.startswith("rad_src_") and os.path.isfile(os.path.join(folder_path, file_name)):
        file_path = os.path.join(folder_path, file_name)
        
        # Ak je súbor prázdny, pridajte tam riadky
        if os.path.getsize(file_path) == 0:
            with open(file_path, 'w') as f:
                f.write("4.71383,-4.96041,0,\n")
                f.write("-6.95004,2.68731,0,\n")
                f.write("6.6954,5.28009,0,\n")
