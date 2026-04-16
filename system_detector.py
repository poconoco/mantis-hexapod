import board

if hasattr(board, "I2C"):
    DESKTOP = False
else:
    DESKTOP = True
