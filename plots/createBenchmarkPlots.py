#!/usr/bin/python

import csv, sys, re;

from common import *;
from matplotlib import *;
from scipy import *;
from pylab import *;

gTToSecFactor = 1.0/3.8 

color={
    'HandOptimizedFunction' : hand_color, 
    'CeresAutoDiffCostFunction' : ceres_color, 
    'TypedExpressionSolver' : tex_color,
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
   'HandOptimizedFunction' : hand_name, 
   'TypedExpressionSolver' : tex_name, 
   'CeresAutoDiffCostFunction' : ceres_name,
}

reader = csv.reader(open("../benchmark/nrotations_1..20.csv", "rb"));

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


for i in range(1, 20) :
    print i, ":", data['CeresAutoDiffCostFunction']['JT'][i][0] / data['TypedExpressionSolver']['JT'][i][0];
    print i, ":", data['TypedExpressionSolver']['JT'][i][0] / data['HandOptimizedFunction']['JT'][i][0];



for timeId in ('JT', ) :
    fig, axes = plt.subplots(figsize = (8, 4));
    axes.set_autoscaley_on(False)
    axes.set_ylim([0,0.15])
    axes.set_xlim([1,20])
#     axes.set_autoscaley_on(True)
    
    for name, d in data.iteritems() :
        dd = d[timeId];
        if not color[name] : continue
        a = numpy.array([ (x[0], x[1][0], x[1][1]) for x in list(sorted(dd.items()))]).transpose();
        #axes.plot(a[0], a[2], color[name], linestyle=linestyle[timeId + "max"], label = ("%s" % (title[name])))
        axes.plot(a[0], a[1], color[name], linestyle=linestyle[timeId], label = ("%s" % (title[name])))
        
    axes.set_xlabel('N')
    axes.set_ylabel('t in seconds')
    axes.legend(loc='upper left')
    savefig('../../isrr13-optimization/article/figures/N_rotations.pdf')
show()
