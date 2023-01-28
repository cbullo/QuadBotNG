import math
import numpy as np
import scipy.interpolate
import scipy.optimize
import matplotlib.pyplot as plt
import pwlf


def filter_data(data, amp):
    fft_data = np.fft.fft(data)
    filtered_fft_data = np.where(np.abs(fft_data) < amp, 0, fft_data)
    # print(np.count_nonzero(filtered_fft_data))
    #print(np.argwhere(np.abs(filtered_fft_data) > 0))
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

with open('data/motor_1.calib') as f:
    lines = f.read().splitlines()

tb = -100
for l in lines:
    abc = l.split(",")
    if tb == float(abc[1]):
        continue
    ta = float(4096 - int(abc[0]) * 64)
    tb = float(abc[1])
    a.append(ta)
    b.append(tb)

a = np.array(a)
b = np.array(b)
a = np.flip(a)
b = np.flip(b)

while np.argmin(b) != 0 and np.argmax(b) != 0:
    #    a = np.roll(a, 1)
    b = np.roll(b, 1)


a = np.insert(a, 0, 0)
#a = np.append(a, 4096 + 64)

b = np.insert(b, 0, b[-1] - 4096)
# b = np.append(b, b[1] + 4096)

print(a)
print(b)

error = a - b
org_error = error
#error = filter_data(error, 300)

smooth_b = a - error

ax1 = plt.subplot(221)
ax2 = plt.subplot(222)
ax3 = plt.subplot(212)

ax1.set_aspect('equal')

# your desired line segment end locations
x0 = np.array(range(0, 4096, int(4096/16)))

# initialize piecewise linear fit with your x and y data
my_pwlf2 = pwlf.PiecewiseLinFit(b, error)

# fit the data with the specified break points
# (ie the x locations of where the line segments
# will terminate)
my_pwlf2.fit_with_breaks(x0)

# predict for the determined points
xHat2 = np.linspace(0, 4096, num=10000)
yHat2 = my_pwlf2.predict(xHat2)


rounded_values = np.round(my_pwlf2.predict(x0)).astype(int)
offset = np.min(rounded_values)
if offset >= 0:
    offset = 0
else:
    offset = -offset

rounded_values = rounded_values + offset
print("LINEARIZATION FACTORS:")
print(rounded_values)
print("OFFSET:")
print(offset)



ax1.plot(a, b, "r", label="original")

# ax1.plot(a, smoothed_b, "g", label="original")

#ax2.plot(a, b + fft_f(b), "r")

# ax3.plot(b, org_error, "r", label="err")
# ax3.plot(a_N, f(a_N), "b", label="err")
# ax3.plot(a, poly_bb_at_a, "b", label="err")
# ax3.plot(b, error-org_error, "g")
# ax3.plot(b[1:-2], fft_f(b[1:-2]) - org_error[1:-2], "g")
# ax3.plot(a_N, fft_at_a, "b", label="err")
# ax3.plot(smooth_b, error, "b", label="err")
# ax3.plot(smooth_b, fft_f(smooth_b), "g", label="err")
# ax3.plot(b[1:-2], fft_rem, "g")


# ax3.plot(a_N, f_a(a_N) - lin_int_hr)


#ax3.plot(a_N, lin_int_hr)
#ax3.plot(a_N, fft_at_a)

#print(*p)
#xd = np.array(range(0, 4096, 4))
#vd = piecewise_linear(xd, *p)
#print(xd)
#print(vd)
#ax3.plot(b, error, "o")
#ax3.plot(xd, piecewise_linear(xd, *p))

#ax3.plot(b, error, "o")
#ax3.plot(xHat, yHat, '-')
#ax3.plot(b, error - my_pwlf.predict(b))

#ax3.plot(b, error, "og")
ax3.plot(xHat2, yHat2)
ax3.plot(x0, np.round(my_pwlf2.predict(x0)), "o")


plt.show()
