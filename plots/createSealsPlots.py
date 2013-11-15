#!/usr/bin/python

import csv;

from matplotlib import *;
from scipy import *;
from pylab import *;

gTToSecFactor = 1.0/3.8 

color={
       'ceres' : 'y', 
       'ceres_m' : '', 
       'ceres_mp' : 'r', 
       'tex' : 'b',
       'tex_m' : '',
       'tex_mp' : 'g',
       }
linestyle={
           'RT' : 'dotted', 
           'RTmax' : '-.', 
           'JT' : '-',
           'JTmax' : 'dashed'
        }


data = {'ceres' : dict(), 'tex' : dict(), 'tex_m' : dict(), 'ceres_m' : dict(), 'tex_mp' : dict(), 'ceres_mp' : dict()}
title = {'ceres' : "ceres", 'tex' : 'tex', 'tex_m' : 'tex_m', 'ceres_m' : 'ceres_m', 'tex_mp' : 'tex_calc', 'ceres_mp' : 'ceres_calc'}
timeTitle = {
           'RT' : 'Residuals', 
           'RTmax' : 'Residuals Max', 
           'JT' : 'Jacobians',
           'JTmax' : 'Jacobians Max'
}

reader = csv.reader(open("../seals/stat.csv", "rb"));

def update(d, timeId, N, val):
    if not val : return 
    if not d.has_key(timeId) :
        d[timeId] = {}
    if not d[timeId].has_key(N) :
        d[timeId][N] = (val, val)
    else :
        d[timeId][N] = (min(d[timeId][N][0], val), max(d[timeId][N][1], val)) 

for row in list(reader) :
    if len(row)  == 5 :
        (name, N, SEQ, RT, JT) = row
        JTP = 0
    else : 
        (name, N, SEQ, RT, JT, RTP, JTP) = row

    N = int(N)
    if RT : RT = float(RT)
    if JT : JT = float(JT)
    if RTP : RTP = float(RTP)
    if JTP : JTP = float(JTP)
    update(data[name], 'RT', N, RT)
    update(data[name], 'JT', N, JT)
    if JTP:
        update(data[name+ "p"] , 'RT', N, RTP * gTToSecFactor)
        update(data[name+ "p"] , 'JT', N, JTP * gTToSecFactor)

fig, axes = plt.subplots(figsize = (20, 10));

for name, d in data.iteritems() :
    for timeId, dd in sorted(d.iteritems()) :
        if not color[name] : continue
        a = numpy.array([ (x[0], x[1][0], x[1][1]) for x in list(sorted(dd.items()))]).transpose();
        #axes.plot(a[0], a[2], color[name], linestyle=linestyle[timeId + "max"], label = ("%s:%s" % (title[name], timeTitle[timeId])))
        axes.plot(a[0], a[1], color[name], linestyle=linestyle[timeId], label = ("%s:%s" % (title[name], timeTitle[timeId])))

for i in range(1, 16) :
    i *= 10000;
    print i, ":", data['ceres_mp']['JT'][i][0] / data['tex_mp']['JT'][i][0];

axes.set_xlabel('N')
axes.set_ylabel('t')
axes.set_title('title');
axes.legend(loc='upper left')
savefig('seals_optimization_stat.pdf')
show()
