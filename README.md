# swarming_notebook

Python notebook to get some initial explanation on swarm robotics and play with a simple swarming algorithm.

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/guidoAI/swarming_notebook/HEAD)

# How to install and run locally
The easiest way to install and run the Jupyter notebook locally is to:
(1) Create a virtual environment for Python (venv) (see, e.g., [this explanation](https://docs.python.org/3/library/venv.html) for Windows ).
(2) Install the required packages with the help of the requirements.txt file (see, e.g., [here](https://stackoverflow.com/questions/7225900/how-can-i-install-packages-using-pip-according-to-the-requirements-txt-file-from) how to do that).
(3) Add the environment as a Kernel to Jupyter notebook (see, e.g., [this blog](https://janakiev.com/blog/jupyter-virtual-envs/#add-virtual-environment-to-jupyter-notebook)).

On Windows in the (GIT) command window:

```
git clone git@github.com:guidoAI/swarming_notebook.git
python -m venv swarming_notebook
cd evolve_cart
.\Scripts\activate.bat
pip install -r requirements.txt
pip install ipykernel
python -m ipykernel install --user --name=swarming
pip install notebook
jupyter notebook
```

Then, in the Jupyter notebook, select ``swarming`` as a Kernel.
