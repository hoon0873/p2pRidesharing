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


def main(n=5, DD = 2, PRECISION = 30, CAP = 4,rhoo=0):
    R = n
    D = DD
    reqs = []
    drivers = []
    T = Distance((0,0), (0.7*PRECISION, 0.7*PRECISION))+PRECISION
    
    LAMBDA = 1
    BIGNUMBER = 5
    MUTE = 1

##    timeF = open("data\SW22SWvsEffEFF.txt", 'a+')
##    timeF.seek(0)    

    BEGINTIME = time.clock()
    print("#DRIVERS",D)
    ACNT = 0
    BCNT = 0
    CCNT = 0
    DCNT = 0
    ECNT = 0

    DCDEV = 1
    DCDET = 6
    RCDEV = 1
    RCDET = 3
    MAXDEV = 10
        
    for i in range(D):
        NEIGHBORHOOD = (int(rd.uniform(0.45,0.55)*PRECISION),int(rd.uniform(0.45,0.55)*PRECISION))
        ori = NEIGHBORHOOD

        randPart = rd.uniform(0,1)

        dWORK = (int(rd.uniform(0.0,0.1)*PRECISION),int(rd.uniform(0.0,0.1)*PRECISION))
        dp = 0.4
        aWORK = (int(rd.uniform(0.7,1.0)*PRECISION),int(rd.uniform(0.7,1.0)*PRECISION))
        pa = 0.2
        bWORK = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        pb = 0.1
        cWORK = (int(rd.uniform(0.2,0.5)*PRECISION),int(rd.uniform(0.6,0.9)*PRECISION))
        pc = 0.2



        # 40% GO TO dWORK
        if randPart <= dp:
            des = dWORK
            
            tim = int(rd.uniform(0,max(T//10,1))) # EARLY WINDOW
            delta = int(rd.uniform(0,0.5)*PRECISION) 
            etim = min(max(T//10,1)+Distance(ori,des),T-1) #STRICT WINDOW
            ptim = max(T//10,1) #STRICT PTIME
            cdev = DCDEV
            cdet = DCDET
            cap = CAP
            val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
            rho = rhoo
            ACNT+=1
        # 20 % GO TO aWork
        elif randPart <= dp+pa:
            des = aWORK
            
            tim = int(rd.uniform(0,max(T//6,1))) # EARLYISH WINDOW
            delta = int(rd.uniform(0,0.5)*PRECISION) 
            etim = min(max(T//6,1)+Distance(ori,des),T-1) #STRICT WINDOW
            ptim = tim+delta//2 #less strict PTIME
            cdev = DCDEV
            cdet = DCDET
            cap = CAP
            val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
            rho = rhoo
            BCNT+=1
        # 10% GO TO SCHOOL
        elif randPart <= dp+pa+pb:
            des = bWORK
            
            tim = int(rd.uniform(0,max(T//3,1))) # EARLYISH WINDOW
##            delta = int(rd.uniform(0,1)*PRECISION) 
            delta = int(rd.gauss(0.5,1)*PRECISION)
            etim = min(tim+delta+Distance(ori,des),T-1) #MORE FLEXIBLE WINDOW
            ptim = tim+delta//2 #less strict PTIME
            cdev = DCDEV
            cdet = DCDET
            cap = CAP
            val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
            rho = rhoo
            CCNT+=1

        elif randPart <= dp+pa+pb+pc:
            des = cWORK
            
            tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
            delta = int(rd.uniform(0,1)*PRECISION)
            etim = min(tim+delta+Distance(ori,des),T-1)
            ptim = tim+delta//2
            cdev = DCDEV
            cdet = DCDET
            cap = CAP
            val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
            rho = rhoo
            DCNT+=1
        # REST ARE RANDOM
        else:
            des = (int(rd.uniform(0,1.0)*PRECISION),int(rd.uniform(0.0,1.0)*PRECISION))
            
            tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
            delta = int(rd.uniform(0,1)*PRECISION)
            etim = min(tim+delta+Distance(ori,des),T-1)
            ptim = tim+delta//2
            cdev = DCDEV
            cdet = DCDET
            cap = CAP
            val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
            rho = rhoo
            ECNT+=1

        maxdev = MAXDEV
            
            
        drivers.append(Driver(ori,des,tim,etim,ptim,cap,cdev,cdet,val,rho,maxdev))
        print('drivers.append(Driver(',ori,",",des,",",tim,",",etim,","
              ,ptim,",",cap,",",cdev,",",cdet,",",val,",",rho,",",maxdev,'))')
    print('# COUNTS: ',ACNT,BCNT,CCNT,DCNT,ECNT)
    print("#REQUESTS",R)     

    ACNT = 0
    BCNT = 0
    CCNT = 0
    DCNT = 0
    ECNT = 0              
    for i in range(R): 
        NEIGHBORHOOD = (int(rd.uniform(0.45,0.55)*PRECISION),int(rd.uniform(0.45,0.55)*PRECISION))
        ori = NEIGHBORHOOD

        randPart = rd.uniform(0,1)

        dWORK = (int(rd.uniform(0.0,0.3)*PRECISION),int(rd.uniform(0.0,0.3)*PRECISION))
        dp = 0.4
        aWORK = (int(rd.uniform(0.7,1.0)*PRECISION),int(rd.uniform(0.7,1.0)*PRECISION))
        pa = 0.2
        bWORK = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        pb = 0.1
        cWORK = (int(rd.uniform(0.3,0.55)*PRECISION),int(rd.uniform(0.3,0.55)*PRECISION))
        pc = 0.2

        
        # 40% GO TO dWORK
        if randPart <= dp:
            des = dWORK
            
            tim = int(rd.uniform(0,max(T//10,1))) # EARLY WINDOW
            delta = int(rd.uniform(0,0.5)*PRECISION) 
            etim = min(max(T//10,1)+Distance(ori,des),T-1) #STRICT WINDOW
            ptim = max(T//10,1) #STRICT PTIME
            cdev = RCDEV
            cdet = RCDET
            val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
            lamb = val
            ACNT +=1
            
        # 20 % GO TO aWork
        elif randPart <= dp+pa:
            des = aWORK
            
            tim = int(rd.uniform(0,max(T//6,1))) # EARLYISH WINDOW
            delta = int(rd.uniform(0,0.5)*PRECISION) 
            etim = min(max(T//6,1)+Distance(ori,des),T-1) #STRICT WINDOW
            ptim = tim+delta//2 #less strict PTIME
            cdev = RCDEV
            cdet = RCDET
            val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
            lamb = val
            BCNT +=1
            
        # 10% GO TO bWORK
        elif randPart <= dp+pa+pb:
            des = bWORK
            
            tim = int(rd.uniform(0,max(T//3,1))) # EARLYISH WINDOW
            delta = int(rd.uniform(0,1)*PRECISION) 
            etim = min(tim+delta+Distance(ori,des),T-1) #MORE FLEXIBLE WINDOW
            ptim = tim+delta//2 #less strict PTIME
            cdev = RCDEV
            cdet = RCDET
            val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
            lamb = val
            CCNT +=1

        elif randPart <= dp+pa+pb+pc:
            des = cWORK
            
            tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
            delta = int(rd.uniform(0,1)*PRECISION)
            etim = min(tim+delta+Distance(ori,des),T-1)
            ptim = tim+delta//2
            cdev = RCDEV
            cdet = RCDET
            val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
            lamb = val
            DCNT +=1
            
        # REST ARE RANDOM
        else:
            des = (int(rd.uniform(0,1.0)*PRECISION),int(rd.uniform(0.0,1.0)*PRECISION))
            
            tim = int(rd.uniform(0,max(T/2-2-Distance(ori,des),1)))
            delta = int(rd.uniform(0,1)*PRECISION)
            etim = min(tim+delta+Distance(ori,des),T-1)
            ptim = tim+delta//2
            cdev = RCDEV
            cdet = RCDET
            val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
            lamb = val
            ECNT +=1

        maxdev = MAXDEV
            

        reqs.append(Passenger(ori,des,tim,etim,ptim,cdev,cdet,val,lamb,maxdev))
        print('reqs.append(Passenger(',ori,",",des,",",tim,",",etim,",",ptim,
              ",",cdev,",",cdet,",",val,",",lamb,",",maxdev,'))')
    print('# COUNTS: ',ACNT,BCNT,CCNT,DCNT,ECNT)

    newRho = rhoo
    for i in range(D):
        drivers[i].rho = newRho
        
    m,x,valSW_SW,valEFF_SW,runtime = nrtv3.MatchingRTV(drivers,reqs,RHO=None)
    print(valSW_SW,valEFF_SW)
    print()

    m2,x2,valSW_EFF,valEFF_EFF,runtime2 = nrtv3.MatchingRTV(drivers,reqs,RHO=0)
    print(valSW_EFF,valEFF_EFF)
    print()
##    timeF.write("%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" %(D, R,newRho,valSW_SW,valEFF_SW,valSW_EFF,valEFF_EFF,runtime,runtime2))
##    timeF.seek(0)

    return valSW_SW,valEFF_SW, valSW_EFF,valEFF_EFF, runtime, runtime2

timeF = open("data\ScalabilityWORK.txt", 'a+')
timeF.seek(0)
newRho = 1.2
for D in range(20,101):
    R = 3*D
    for REPEAT in range(3):
        valSW_SW,valEFF_SW, valSW_EFF,valEFF_EFF, runtime, runtime2 = main(R, D, rhoo=newRho)
        timeF.write("%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" %(D, R,newRho,valSW_SW,valEFF_SW,valSW_EFF,valEFF_EFF,runtime,runtime2))
        timeF.seek(0)
timeF.close()

##DD = 2
##n = 4
##val, runtime = main(n,DD)
    
