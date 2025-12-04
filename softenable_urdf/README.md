# SoftEnable URDF

This folder has the SoftEnable URDF along with relevant STLs. For convience a script is provided for visualize it using pinocchio.

To install the dependencies create a virtual environment and do the following:

Inside `softenable_urdf` folder run:

```bash
python3 -m venv .venv --prompt "technion_urdf"
source .venv/bin/activate
pip install -r requirements.txt
python3 visualize_urdf.py
```
The frame for placement planning is called: `placement_link`.