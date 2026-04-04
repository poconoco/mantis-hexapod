import board

from adafruit_pca9685 import PCA9685

class PcaPool:
    def __init__(self, i2c: board.I2C):
        self.i2c = i2c
        self.pca_dict = {}

    def __del__(self):
        for pca in self.pca_dict.values():
            pca.deinit()

    def get_pca(self, address: int) -> PCA9685:
        if address not in self.pca_dict:
            pca = PCA9685(i2c_bus=self.i2c, address=address)
            pca.frequency = 50
            self.pca_dict[address] = pca

        return self.pca_dict[address]
