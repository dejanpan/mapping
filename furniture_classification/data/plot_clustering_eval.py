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
    if fnmatch.fnmatch(file, 'res.sgf.40.*.txt'):
        f = file.split('.')
	print file
	data = np.loadtxt(file, dtype=np.float32)
	plt.plot(1-data[:,4],data[:,5])
	if(len(f) == 4):
		l.append(f[1] + ' ' + f[2] + ' clusters')
	else:
		l.append(f[2] + '.' + f[3])

plt.legend(l, loc=2)
plt.savefig('res.png')
