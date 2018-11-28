import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
##from NewFeasible import travel
from itertools import combinations as COMB
import math
##import NewMatchingRTV as nrtv
##import NewMatchingRTV2 as nrtv2
import NewMatchingRTV3 as nrtv3
from NewFeasibleBruthWTime import Distance
import twoMILP as mlp
import twoMILPPrun as mlp2
import DecompMILPPrun as mlp3


timeF = open("TimeTest.txt", 'a+')
timeF.seek(0)

reqs = []
drivers = []


LAMBDA = 1
BIGNUMBER = 5
MUTE = 1

##    timeF = open("SW22SWvsEffEFF.txt", 'a+')
##    timeF.seek(0)    

BEGINTIME = time.clock()
#DRIVERS 12
drivers.append(Driver( (3, 1) , (19, 20) , 0 , 60 , 0 , 4 , 1 , 6 , 269 , 1.2 ))
drivers.append(Driver( (25, 27) , (14, 16) , 0 , 60 , 0 , 4 , 1 , 6 , 201 , 1.2 ))
drivers.append(Driver( (3, 1) , (19, 20) , 0 , 60 , 0 , 4 , 1 , 6 , 269 , 1.2 ))
drivers.append(Driver( (29, 28) , (15, 19) , 0 , 60 , 0 , 4 , 1 , 6 , 187 , 1.2 ))
drivers.append(Driver( (1, 2) , (18, 20) , 0 , 60 , 0 , 4 , 1 , 6 , 295 , 1.2 ))
##drivers.append(Driver( (5, 4) , (12, 16) , 0 , 60 , 0 , 4 , 1 , 6 , 151 , 1.2 ))
##drivers.append(Driver( (25, 26) , (17, 13) , 0 , 60 , 0 , 4 , 1 , 6 , 152 , 1.2 ))
##drivers.append(Driver( (4, 1) , (17, 18) , 0 , 60 , 0 , 4 , 1 , 6 , 208 , 1.2 ))
##drivers.append(Driver( (28, 24) , (16, 20) , 0 , 60 , 0 , 4 , 1 , 6 , 162 , 1.2 ))
##drivers.append(Driver( (2, 5) , (14, 16) , 0 , 60 , 0 , 4 , 1 , 6 , 140 , 1.2 ))
##drivers.append(Driver( (29, 24) , (17, 12) , 0 , 60 , 0 , 4 , 1 , 6 , 173 , 1.2 ))
##drivers.append(Driver( (0, 2) , (15, 18) , 0 , 60 , 0 , 4 , 1 , 6 , 282 , 1.2 ))
##drivers.append(Driver( (24, 24) , (14, 16) , 0 , 60 , 0 , 4 , 1 , 6 , 170 , 1.2 ))
#REQUESTS 36
reqs.append(Passenger( (5, 4) , (16, 20) , 5 , 36 , 10 , 1 , 3 , 121 , 121 ))
reqs.append(Passenger( (26, 28) , (20, 14) , 0 , 29 , 6 , 1 , 3 , 103 , 103 ))
reqs.append(Passenger( (2, 1) , (16, 15) , 3 , 24 , 3 , 1 , 3 , 116 , 116 ))
reqs.append(Passenger( (26, 28) , (20, 18) , 11 , 46 , 22 , 1 , 3 , 80 , 80 ))
reqs.append(Passenger( (29, 27) , (15, 17) , 5 , 27 , 7 , 1 , 3 , 120 , 120 ))
reqs.append(Passenger( (3, 2) , (16, 12) , 9 , 37 , 14 , 1 , 3 , 107 , 107 ))
reqs.append(Passenger( (4, 3) , (16, 13) , 5 , 49 , 19 , 1 , 3 , 87 , 87 ))
reqs.append(Passenger( (25, 26) , (14, 18) , 9 , 29 , 12 , 1 , 3 , 102 , 102 ))
reqs.append(Passenger( (5, 2) , (18, 14) , 5 , 24 , 5 , 1 , 3 , 83 , 83 ))
reqs.append(Passenger( (26, 27) , (17, 12) , 1 , 36 , 9 , 1 , 3 , 122 , 122 ))
reqs.append(Passenger( (4, 0) , (13, 19) , 2 , 46 , 13 , 1 , 3 , 136 , 136 ))
reqs.append(Passenger( (27, 28) , (13, 15) , 2 , 46 , 14 , 1 , 3 , 148 , 148 ))
reqs.append(Passenger( (5, 2) , (13, 20) , 0 , 47 , 13 , 1 , 3 , 129 , 129 ))
reqs.append(Passenger( (25, 25) , (16, 20) , 2 , 29 , 10 , 1 , 3 , 74 , 74 ))
##reqs.append(Passenger( (1, 2) , (16, 20) , 0 , 25 , 0 , 1 , 3 , 129 , 129 ))
##reqs.append(Passenger( (28, 25) , (20, 20) , 6 , 36 , 16 , 1 , 3 , 46 , 46 ))
##reqs.append(Passenger( (2, 1) , (16, 12) , 9 , 27 , 9 , 1 , 3 , 89 , 89 ))
##reqs.append(Passenger( (27, 24) , (12, 20) , 0 , 33 , 8 , 1 , 3 , 74 , 74 ))
##reqs.append(Passenger( (5, 5) , (20, 15) , 7 , 49 , 18 , 1 , 3 , 103 , 103 ))
##reqs.append(Passenger( (25, 26) , (20, 20) , 1 , 32 , 12 , 1 , 3 , 45 , 45 ))
##reqs.append(Passenger( (0, 0) , (13, 15) , 4 , 51 , 17 , 1 , 3 , 113 , 113 ))
##reqs.append(Passenger( (24, 28) , (18, 20) , 11 , 26 , 13 , 1 , 3 , 72 , 72 ))
##reqs.append(Passenger( (4, 2) , (17, 16) , 4 , 31 , 7 , 1 , 3 , 127 , 127 ))
##reqs.append(Passenger( (28, 25) , (14, 16) , 0 , 41 , 12 , 1 , 3 , 96 , 96 ))
##reqs.append(Passenger( (3, 3) , (18, 14) , 8 , 48 , 18 , 1 , 3 , 103 , 103 ))
##reqs.append(Passenger( (24, 28) , (13, 19) , 0 , 40 , 12 , 1 , 3 , 81 , 81 ))
##reqs.append(Passenger( (5, 2) , (20, 14) , 3 , 23 , 3 , 1 , 3 , 119 , 119 ))
##reqs.append(Passenger( (29, 25) , (17, 18) , 4 , 18 , 4 , 1 , 3 , 95 , 95 ))
##reqs.append(Passenger( (1, 4) , (17, 16) , 2 , 42 , 12 , 1 , 3 , 95 , 95 ))
##reqs.append(Passenger( (28, 24) , (17, 17) , 13 , 37 , 18 , 1 , 3 , 94 , 94 ))
##reqs.append(Passenger( (2, 4) , (20, 15) , 4 , 44 , 13 , 1 , 3 , 121 , 121 ))
##reqs.append(Passenger( (29, 25) , (20, 18) , 8 , 38 , 17 , 1 , 3 , 84 , 84 ))
##reqs.append(Passenger( (5, 3) , (17, 15) , 10 , 32 , 12 , 1 , 3 , 127 , 127 ))
##reqs.append(Passenger( (26, 29) , (17, 16) , 11 , 37 , 16 , 1 , 3 , 101 , 101 ))
##reqs.append(Passenger( (3, 0) , (17, 19) , 2 , 30 , 4 , 1 , 3 , 110 , 110 ))
##reqs.append(Passenger( (27, 28) , (19, 16) , 4 , 25 , 7 , 1 , 3 , 106 , 106 ))
D = len(drivers)
R = len(reqs)
newRho = 1.2
print(len(drivers),len(reqs))
    
m,x,valSW_SW,valEFF_SW,runtime = nrtv3.MatchingRTV(drivers,reqs,RHO=None)
print(valSW_SW,valEFF_SW)
print()

##m2,x2,valSW_EFF,valEFF_EFF,runtime2 = nrtv3.MatchingRTV(drivers,reqs,RHO=0)
##print(valSW_EFF,valEFF_EFF)
##print()

##return valSW_SW,valEFF_SW, valSW_EFF,valEFF_EFF, runtime, runtime2

##timeF.write("%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" %(D, R,newRho,valSW_SW,valEFF_SW,valSW_EFF,valEFF_EFF,runtime,runtime2))
timeF.close()

##DD = 2
##n = 4
##val, runtime = main(n,DD)
    
