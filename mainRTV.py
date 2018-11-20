import numpy as np
import networkx as nx
import pylab
import time
import random as rd
import gurobipy as grb
from Classes import Driver
from Classes import Passenger
from NewFeasible import travel
from itertools import combinations as COMB
import math
import NewMatchingRTV as nrtv
import NewMatchingRTV2 as nrtv2
import NewMatchingRTV3 as nrtv3
from NewFeasible import Distance
import twoMILP as mlp
import twoMILPPrun as mlp2
import DecompMILPPrun as mlp3


inpF = open("data\inputRTV.txt", 'a+')

# COMPARING SINGLE MILP, RTV+MILP; RTV+SEARCH
def main(n=20, DD = 2, PRECISION = 30, CAP = 4):
    R = n
    D = DD
    reqs = []
    drivers = []
    ##T = 150
##    T = Distance((0,0), (0.7*PRECISION, 0.7*PRECISION))+PRECISION
##    T = Distance((0,0),(0.7*PRECISION,0.7*PRECISION))+5
    T = Distance((0,0),(1*PRECISION,1*PRECISION))+PRECISION//2
    LAMBDA = 1
    BIGNUMBER = 5
    MUTE = 1

    BEGINTIME = time.clock()
    print("\n\nDRIVERS",D)
    inpF.write("%d\t%d\n" %(D,R))
    for i in range(D):
##        ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##        des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##        if i%2 == 0:
##            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
##        if i%2 == 1:
##            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
##        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))
        ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        des = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
        tim = 0
        etim = T
    ##    delta = int(rd.uniform(0,1)*PRECISION)
    ##    etim = min(tim+delta+Distance(ori,des),T-1)
        cap = CAP
        drivers.append(Driver(ori,des,tim,etim,cap))
        inpF.write('drivers.append(Driver('+str(ori)+","+str(des)+","+str(tim)+","+str(etim)+","+str(cap)+'))')
        inpF.write("\n")
##        inpF.seek(0)
##        print('drivers.append(Driver(',ori,",",des,",",tim,",",etim,",",cap,'))')

    print("REQUESTS",R)                   
    for i in range(R): 
##        ori = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##        des = (int(rd.uniform(0,1)*PRECISION),int(rd.uniform(0,1)*PRECISION))
##        if i%2 == 0:
##            ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
##        if i%2 == 1:
##            ori = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))
##        des = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0.4,0.7)*PRECISION))
##        F = i%5
##        ori = (int(rd.uniform(F/6,(F+1)/6)*PRECISION),int(rd.uniform(F/6,(F+1)/6)*PRECISION))
##        des = (int(rd.uniform((F+1)/6,(F+2)/6)*PRECISION),int(rd.uniform((F+1)/6,(F+2)/6)*PRECISION))
        ori = (int(rd.uniform(0,0.2)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        des = (int(rd.uniform(0.8,1)*PRECISION),int(rd.uniform(0.8,1)*PRECISION))

        
        tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
        delta = int(rd.uniform(0,1)*PRECISION)
        etim = min(tim+delta+Distance(ori,des),T-1)
        ptim = tim+delta//2
    ##    tim = 0
    ##    etim = T-1
        reqs.append(Passenger(ori,des,tim,etim,ptim))
        inpF.write('reqs.append(Passenger('+str(ori)+","+str(des)+","+str(tim)+","+str(etim)+","+str(ptim)+'))')
##        print('reqs.append(Passenger(',ori,",",des,",",tim,",",etim,",",ptim,'))')

        inpF.write("\n")
##        inpF.seek(0)


    inpF.write("\n")
    inpF.seek(0)

# COMPARING SINGLE MILP, RTV+MILP; RTV+SEARCH


    print("\nFIRST\n")

##    val,runtime = mlp3.Decomp(drivers,reqs)
##    m,x,val,runtime = nrtv.MatchingRTV(drivers,reqs)
##    m,x,val,runtime, G0, c0 = mlp2.MILP(drivers,reqs)

    val, runtime = 0,0
##

    print("\nSECOND\n")
##    m1,x1,val1,runtime1 = nrtv2.MatchingRTV(drivers,reqs)
##    m1,x1,val1,runtime1 = nrtv2.MatchingRTV(drivers,reqs)
##    P,c = 0,0

##    val1,runtime1= mlp3.Decomp(drivers,reqs)
##    P,c = 0,0

##    m1,x1,val1,runtime1,P,c = mlp2.MILP(drivers,reqs)
    
    m1,x1,val1,runtime1,P,c = 0,0,0,0,0,0


    print("\nTHIRD\n")
    m2,x2,val2,runtime2 = nrtv3.MatchingRTV(drivers,reqs)
    
    print(val, val1, val2)
    print(runtime,runtime1,runtime2)
    return runtime, runtime1, runtime2, val,val1, val2
    return runtime, 0, val, 0

timeF = open("data\FeasibleTimeRTV.txt", 'a+')
timeF.seek(0)
DD = 5
PRECISION = 30
CAP = 4
R = 20
for i in range(10,100,10):
    R=i
    r1, r2, r3, v1,v2, v3 = main(R, DD, PRECISION, CAP)
    timeF.write("%d\t%d\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\n" %(DD, R, PRECISION, CAP,r1,v1,r2,v2,r3,v3))
    timeF.seek(0)
timeF.close()


