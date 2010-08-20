import os, sys
from numpy import *
import matplotlib.pyplot as plt
from pylab import *

conf_array_array = []

def buildConfusionMatrix(path,nr):
    for i in range(1, int(nr)):
        input = open(path)
        #print "range: ", i
        k = l = 0
        nr_lines = 0
        conf_array = []
        for el in range(1, int((nr))):
            conf_array.append(0)
        for line in input.xreadlines():
            #print "line: ", line.split()
            j = [int(line.split()[0])/10, int(line.split()[1])/10]
            if i == int(j[1]):
               conf_array[int(j[0]) - 1] =  conf_array[int(j[0]) - 1] + 1
            #nr of lines
            nr_lines = nr_lines + 1
        print "nr lines", nr_lines
        #print conf_array
        conf_array_array.append(conf_array)
        input.close()
    print 'rows are ground truth, columns the classification: ', conf_array_array
    compute_false_pos_statistics(conf_array_array)
    plot_confusion_matrix(conf_array_array, int(nr)-1)

def plot_confusion_matrix(conf_arr, nr):
    norm_conf = []
    for i in conf_arr:
        a = 0
        tmp_arr = []
        a = sum(i,0)
        for j in i:
		if j == 0 and a == 0:
			tmp_arr.append(0)
		else:
			tmp_arr.append(float(j)/float(a))
		#tmp_arr.append(j)
        norm_conf.append(tmp_arr)

    plt.clf()
    fig = plt.figure()
    plt.xticks( arange(nr), ('SmallCylinder',  'FlatBigBox',  'TetraPak',  'MediumBox',  'Teapot',  'TallCylinder'), rotation=90)
    plt.yticks( arange(nr), ('SmallCylinder',  'FlatBigBox',  'TetraPak',  'MediumBox',  'Teapot',  'TallCylinder') )
    plt.title('GRSD Confusion Matrix for Training Objects')
    ax = fig.add_subplot(111)
    res = ax.imshow(array(norm_conf),  cmap=cm.jet, interpolation='nearest', aspect='auto')
    cb = fig.colorbar(res)
    savefig("confmat.png", format="png", bbox_inches='tight')

def compute_false_pos_statistics(conf_arr):
    tp = 0
    all = 0
    array_len = conf_arr.__len__()
    for i in range(array_len):
        tp = tp + conf_arr[i][i]
        all = all + sum(conf_arr[i])
    print 'all: ', all, 'true positives: ', tp, 'percentage: ', double(tp)/double(all)


if __name__ == "__main__":
    if sys.argv.__len__() < 3:
        print "Usage: build_confusion_matrix.py \t test.conf \t number_of_classes+1 \n\nAlso, adopt lines plt.xticks plt.yticks with actual category names"
        sys.exit()
    path = sys.argv[1]
    nr = sys.argv[2]
    buildConfusionMatrix(path, nr)
