import numpy as np
import matplotlib.pyplot as plt
from pynufft import NUFFT

def filter_data(data):
    fft_data = np.fft.fft(data)
    filtered_fft_data = np.where(np.abs(fft_data) < 100, 0, fft_data)
    #filtered_fft_data = fft_data
    #filtered_fft_data[50:] = 0
    print(np.count_nonzero(filtered_fft_data))
    print(np.argwhere(np.abs(filtered_fft_data) > 0))
    return np.real(np.fft.ifft(filtered_fft_data))

a = []    
b = []
c = []

with open('data/motor_1') as f:
  lines = f.read().splitlines()

for l in lines:
  abc = l.split()
  a.append(float(abc[0]))
  b.append(float(abc[1]))
  c.append(float(abc[2]))

a = np.array(a)
b = np.array(b)
c = np.array(c)

filtered_values = filter_data(c)

#plt.plot(a, c, "g", label="original")
plt.plot(a, a - (b + filtered_values), "r", label="true")
#plt.plot(a, filtered_values, "b", label="filtered")

plt.show()
