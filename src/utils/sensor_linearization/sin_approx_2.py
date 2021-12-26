import math
import numpy as np
import scipy.interpolate
import matplotlib.pyplot as plt

def filter_data(data, amp):
    fft_data = np.fft.fft(data)
    filtered_fft_data = np.where(np.abs(fft_data) < amp, 0, fft_data)
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

with open('data/motor_0.calib') as f:
    lines = f.read().splitlines()

tb = -100
for l in lines:
    abc = l.split(",")
    if tb == float(abc[1]): continue
    ta = float(abc[0])
    tb = float(abc[1])
    a.append(ta)
    b.append(tb)

a = np.array(a)
b = np.array(b)

error = a - b
org_error = error
error = filter_data(error, 300)

smooth_b = a - error

ax1 = plt.subplot(221)
ax2 = plt.subplot(222)
ax3 = plt.subplot(212)

ax1.set_aspect('equal')
# ax2.set_aspect('equal')

#poly_bb = np.polyfit(b, error, 11)

N = 4096
f = scipy.interpolate.UnivariateSpline(smooth_b, error, k=3, s=1)
a_N = np.arange(0, 360, 360/N)

fft_at_a = filter_data(f(a_N), 1000)
fft_f = scipy.interpolate.UnivariateSpline(a_N, fft_at_a, s=0)

f_a = scipy.interpolate.UnivariateSpline(smooth_b, a, k=3, s=0)

l_N = np.linspace(0, 360, 16)
lin_int = np.interp(l_N, smooth_b, a)
lin_int_hr = np.interp(a_N, l_N, lin_int)

err_lin_int_hr = np.interp(a_N, l_N, f(l_N))

#fft_rem = filter_data(fft_f(b[1:-2]) - org_error[1:-2], 50)

#fft_at_a = np.fft.ifft(error_fft)
#fft_at_a = finufft.nufft1d2(a_N, error_fft)
#print(error_fft)

#poly_bb_at_a = np.polyval(poly_bb, a)
#poly_bb_at_b = np.polyval(poly_bb, b)

ax1.plot(a, b, "r", label="original")
#ax1.plot(a, smoothed_b, "g", label="original")

ax2.plot(a, b + fft_f(b), "r")

#ax3.plot(b, org_error, "r", label="err")
#ax3.plot(a_N, f(a_N), "b", label="err")
#ax3.plot(a, poly_bb_at_a, "b", label="err")
#ax3.plot(b, error-org_error, "g")
#ax3.plot(b[1:-2], fft_f(b[1:-2]) - org_error[1:-2], "g")
#ax3.plot(a_N, fft_at_a, "b", label="err")
#ax3.plot(smooth_b, error, "b", label="err")
#ax3.plot(smooth_b, fft_f(smooth_b), "g", label="err")
#ax3.plot(b[1:-2], fft_rem, "g")


#ax3.plot(a_N, f_a(a_N) - lin_int_hr)
ax3.plot(a_N, f(a_N) - err_lin_int_hr)
ax3.plot(a_N, f(a_N) - fft_at_a)

plt.show()
