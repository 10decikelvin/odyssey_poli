# Installation

To avoid system pollution, always remember to use `venv`.

```bash
# assuming you are in root
cd inference
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

# Usage

```bash
python webcam_lsd.py
```

or if you want to specify the path to the model and the size of the input:

```bash
python webcam_lsd.py --path /path/to/model --size 320
```

`webcam_lsd.py` also supports the following arguments:

- `--path` to specify the path to the model file
- `--size` size of the input (320 or 512)