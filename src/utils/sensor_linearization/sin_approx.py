import math
import numpy as np
import matplotlib.pyplot as plt
import rdp as rdp
from scipy import interpolate


def filter_data(data):
    fft_data = np.fft.fft(data)
    filtered_fft_data = np.where(np.abs(fft_data) < 1000, 0, fft_data)
    print(np.count_nonzero(filtered_fft_data))
    print(np.argwhere(np.abs(filtered_fft_data) > 0))
    return np.real(np.fft.ifft(filtered_fft_data))


def fft(data):
    fft_data = np.fft.rfft(data)
    filtered_fft_data = np.where(np.abs(fft_data) < 500, 0, fft_data)
    # print(np.count_nonzero(filtered_fft_data))
    # print(np.argwhere(np.abs(filtered_fft_data) > 0))
    # return filtered_fft_data[filtered_fft_data != 0]
    return filtered_fft_data


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

fft_c = filter_data(a - b)

# def eval_fft(x):
#   ret = np.real(fft_c[0]) / 2
#   ret = 0
#   for idx, X in enumerate(fft_c[1:]):
#     ret += np.real(X) * math.sin((idx + 1) * x + np.imag(X))
#   return ret

# eval_fft_v = np.vectorize(eval_fft)

smoothed_b = a - fft_c

ax1 = plt.subplot(221)
ax2 = plt.subplot(222)
ax3 = plt.subplot(212)

ax1.set_aspect('equal')
#ax2.set_aspect('equal')

err_corr = smoothed_b - a

poly_bb = np.polyfit(b, err_corr, 11)
poly_bb_at_a = np.polyval(poly_bb, a)
poly_bb_at_b = np.polyval(poly_bb, b)

ax1.plot(a, b, "r", label="original")
ax1.plot(a, smoothed_b, "g", label="original")

ax2.plot(smoothed_b, a, "r")

ax3.plot(b, a - (b - poly_bb_at_b), "b", label="err")



plt.show()
