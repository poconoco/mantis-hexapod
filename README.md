# Проєкт "Богомол"
# Project "Mantis"

This are the sources for remote controlled hexapod.

Videos about this robot:

- https://youtu.be/ImilFi91mjo
- https://youtu.be/b2wyORER510

Remote control app: https://play.google.com/store/apps/details?id=com.nocomake.serialremote

## Requirements

This is to be runned on raspberry pi or similar system that has access to i2c or in visualization mode on any desktop system.

## Preparation

### Create venv

```
python3 -m venv .venv
```

### Activate venv

```
source .venv/bin/activate
```

### Install dependencies

```
python -m pip install -r requirements.txt
```

## Run the engine

```
python run.py
```

It will automatically detect if you have board.I2C available. If yes - it will use config.yaml for hexapod servo layout configuration and run the movement engine. If I2C is not available it will open matplotlib window with teg tip points shown. You still can connect with remote controller app to it and control those "virtual legs".
