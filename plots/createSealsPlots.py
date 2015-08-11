#!/usr/bin/python

import csv;
from common import *;

from matplotlib import *;
from scipy import *;
from pylab import *;

gTToSecFactor = 1.0/3.8 


color={
       'ceres' : ceres_color, 
       'ceres_m' : '', 
       'ceres_mp' : ceres_color, 
       'tex' : tex_color,
       'tex_m' : '',
       'tex_mp' : tex_color,
       }
linestyle={
           'ceresRT' : ceres_ls, 
           'ceresRTmax' : ceres_ls, 
           'ceresJT' : ceres_ls,
           'ceresJTmax' : ceres_ls,
           'texRT' : tex_ls, 
           'texRTmax' : tex_ls, 
           'texJT' : tex_ls,
           'texJTmax' : tex_ls,
        }


data = {'ceres' : dict(), 'tex' : dict(), 'tex_m' : dict(), 'ceres_m' : dict(), 'tex_mp' : dict(), 'ceres_mp' : dict()}


title = {
 'ceres' : ceres_name,
 'ceres_m' : ceres_name,
 'ceres_mp' : ceres_name,
 'tex' : tex_name,
 'tex_m' : tex_name,
 'tex_mp' : tex_name,
}
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

for i in range(1, 16) :
    i *= 10000;
    print i, ":", data['ceres_m']['JT'][i][0] / data['tex_m']['JT'][i][0];
    print i, ":", data['tex']['JT'][i][0] / data['ceres']['JT'][i][0];


for i in range(1, 16) :
    i *= 10000;
    print i, ":", data['ceres_mp']['JT'][i][0] / data['tex_mp']['JT'][i][0];
    print i, ":", data['tex_mp']['JT'][i][0] / data['ceres_mp']['JT'][i][0];


for timeId in ('JT', 'RT') :
    for suffix in ('', '_mp') :
        fig, axes = plt.subplots(figsize = (4, 4));
        
        for var in ('ceres', 'tex'):
            name = var + suffix
            d = data[name];
            if not color[name] : continue
            dd = d[timeId];
            a = numpy.array([ (x[0], x[1][0], x[1][1]) for x in list(sorted(dd.items()))]).transpose();
#             axes.plot(a[0] / 1000, a[2], color[name], linestyle=linestyle[timeId + "max"], label = ("%s" % (title[name])))
            axes.plot(a[0] / 1000, a[1], color[name], linestyle=linestyle[var + timeId], label = ("%s" % (title[name])))
    
        axes.set_xlabel('N in 1000 input lines')
        axes.set_ylabel('t in seconds')
        axes.legend(loc='upper left')
        grid()
        savefig(outputDir + "seals_optimization_stat%s_%s.pdf" % (suffix, timeId))

show()

