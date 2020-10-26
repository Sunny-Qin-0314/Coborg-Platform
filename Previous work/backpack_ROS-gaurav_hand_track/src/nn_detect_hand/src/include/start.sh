virtualenv --system-site-packages -p python3 ./venv2
source ./venv2/bin/activate  # sh, bash, ksh, or zsh
pip install --upgrade pip
pip install -r requirements.txt
pip install --upgrade tensorflow-gpu

