import numpy as np
x = -5.39170360565
y = 7.05215406418
z = -2.04894518852

angle = np.arctan2(z, np.sqrt(x * x + y * y)) * 180 / np.pi
if angle >= -8.83:
  line = int((2 - angle) * 3.0 + 0.5)
else:
  line = 64 / 2 + int((-8.83 - angle) * 2.0 + 0.5)

print(line)
