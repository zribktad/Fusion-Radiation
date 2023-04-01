import os
import shutil

# Zadajte cestu k adresáru, z ktorého sa majú súbory kopírovať
zdrojovy_priecinok = "/home/tadeas/my_workspace/src/fusion_radiation/data/Final"

# Zadajte cestu k cieľovému priecinku, do ktorého sa majú súbory skopírovať
cielovy_priecinok = "/home/tadeas/my_workspace/src/fusion_radiation/data/All"

# Pre každý priecinok v zdrojovom adresári
for priecinok, podpriecinky, subory in os.walk(zdrojovy_priecinok):
    # Pre každý súbor v priecinku
    for subor in subory:
        # Zloženie cesty k súboru
        zdrojova_cesta = os.path.join(priecinok, subor)
        cielova_cesta = os.path.join(cielovy_priecinok, subor)
        # Kopírovanie súboru do cieľového adresára
        #shutil.copy(zdrojova_cesta, cielova_cesta)
        # Vytvorenie symbolického odkazu
        os.symlink(zdrojova_cesta, cielova_cesta)

