#!/usr/bin/env python
# -*- coding: utf-8 -*-
import csv, sys
import numpy as np
import matplotlib.pyplot as plt

skip = 0
data_matrix = []
with open(sys.argv[1], 'rb') as csvfile:
    data = csv.reader(csvfile, delimiter=',')
    for row in data:
        if skip < 4:
            skip = skip + 1
        else:
            data_matrix.append([float(x) for x in row])

angles = np.array([x[0] for x in data_matrix])
initial_mse = np.array([x[1] for x in data_matrix])
final_mse = np.array([x[2] for x in data_matrix])
score = np.array([x[3] for x in data_matrix])
gain = initial_mse - final_mse

tup = np.column_stack([final_mse, score])
tup = tup.tolist()

mse_sorted = sorted(tup, key = lambda x: x[0])
score_sorted = sorted(tup, key = lambda x: x[1])

i = 0
for mse in mse_sorted:
    if mse[0] != score_sorted[i][0]:
        print "Problem! Order not preserved!"
    i = i + 1

plt.figure(1)
plt.subplot(211)
plt.plot(angles, gain,'bo', angles, final_mse,'ro', angles, initial_mse, 'go' )

plt.subplot(212)
plt.plot(angles, score,'ro')
plt.show()
