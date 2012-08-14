import numpy as np
import matplotlib.pyplot as plt
import os
import fnmatch


plt.ylim([0,1])
plt.xlim([0,1])
plt.xlabel('1 - Precision')
plt.ylabel('Recall')

l = []

for file in os.listdir('.'):
    if fnmatch.fnmatch(file, 'res.*.txt'):
        f = file.split('.')
	data = np.loadtxt(file, dtype=np.float32)
	plt.plot(1-data[:,4],data[:,5])
	l.append(f[1] + ' ' + f[2] + ' clusters')

plt.legend(l, loc=2)
plt.savefig('res.png')
