#!/usr/bin/python

import csv, sys, re;

from matplotlib import *;
from scipy import *;
from pylab import *;

gTToSecFactor = 1.0/3.8 

color={
           'HandOptimizedFunction' : 'y', 
           'CeresAutoDiffCostFunction' : 'r', 
           'TypedExpressionSolver' : 'g',
       }

linestyle={
   'RT' : 'dotted', 
   'RTmax' : '-.', 
   'JT' : '-',
   'JTmax' : 'dashed',
   'PT' : '.',
}

timeTitle = {
           'PT' : 'Preparation', 
           'RT' : 'Residuals', 
           'RTmax' : 'Residuals Max', 
           'JT' : 'Jacobians',
           'JTmax' : 'Jacobians Max'
}


data = {'HandOptimizedFunction' : dict(), 'TypedExpressionSolver' : dict(), 'CeresAutoDiffCostFunction' : dict()}
title = {
   'HandOptimizedFunction' : 'HandOptimizedFunction', 
   'TypedExpressionSolver' : 'TypedExpressionSolver', 
   'CeresAutoDiffCostFunction' : 'CeresAutoDiffCostFunction',
}

reader = csv.reader(open("benchmark_stat.csv", "rb"));

def update(d, timeId, N, val):
    if not val : return 
    if not d.has_key(timeId) :
        d[timeId] = {}
    if not d[timeId].has_key(N) :
        d[timeId][N] = (val, val)
    else :
        d[timeId][N] = (min(d[timeId][N][0], val), max(d[timeId][N][1], val)) 


newProblemPattern = re.compile("NEW PROBLEM\(rotation_problem\(N=(.*)\)\)") 

N = 0;
for row in list(reader) :
    if(len(row) == 0) : continue
    match = newProblemPattern.match(row[0])
    if match:
        N = int(match.group(1))
        print N
        continue
    
    name = row[0].split('[')[0]
    
    if N and title.has_key(name) :
        print name;
        print row
        (nameComplete, PT, MEMSIZE, RT, JT) = row[0:5]

        print MEMSIZE    
        MEMSIZE = int(MEMSIZE)
        if PT : PT = float(PT)
        if RT : RT = float(RT)
        if JT : JT = float(JT)
        update(data[name], 'RT', N, RT)
        update(data[name], 'JT', N, JT)
        update(data[name], 'PT', N, PT)

print data

fig, axes = plt.subplots(figsize = (20, 10));

for name, d in data.iteritems() :
    for timeId, dd in sorted(d.iteritems()) :
        if not color[name] : continue
        a = numpy.array([ (x[0], x[1][0], x[1][1]) for x in list(sorted(dd.items()))]).transpose();
        #axes.plot(a[0], a[2], color[name], linestyle=linestyle[timeId + "max"], label = ("%s:%s" % (title[name], timeTitle[timeId])))
        axes.plot(a[0], a[1], color[name], linestyle=linestyle[timeId], label = ("%s:%s" % (title[name], timeTitle[timeId])))
        
axes.set_xlabel('N')
axes.set_ylabel('t')
axes.set_title('title');
axes.legend(loc='upper left')
savefig('seals_optimization_stat.pdf')
show()
