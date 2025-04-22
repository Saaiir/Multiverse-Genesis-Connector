# Multiverse-Genesis-Connector

```bash

    git clone git@github.com:Multiverse-Framework/Multiverse-Genesis-Connector.git
    cd Multiverse-Genesis-Connector
    git submodule update --init --depth 1 resources/mjcf/mujoco_menagerie
    . $(which virtualenvwrapper.sh)
    mkvirtualenv genesis
    python3 -m pip install -U pip
    python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu126
    python3 -m pip install -r requirements.txt

```