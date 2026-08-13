import os
import shutil
from esphome.core import CORE

# ... tvůj předchozí kód komponenty ...

# 1. Zdrojová cesta k fyzickým souborům Eigen z tvého Gitu
component_dir = os.path.dirname(__file__)
source_eigen = os.path.join(component_dir, "Eigen")

# 2. Cílová cesta v kořeni buildu (src/Eigen), na kterou vidí celý ESP-IDF
dest_eigen = CORE.relative_build_path("src", "Eigen")

# 3. Hrubé zkopírování celé složky včetně souborů bez přípon
if os.path.exists(source_eigen):
    if os.path.exists(dest_eigen):
        shutil.rmtree(dest_eigen)
    shutil.copytree(source_eigen, dest_eigen)