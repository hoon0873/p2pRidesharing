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
import time

def main(n=5, DD = 2, PRECISION = 30, CAP = 4,rhoo=0):
    R = n
    D = DD
    reqs = []
    drivers = []
    T = 2*Distance((0,0), (1*PRECISION, 1*PRECISION))
    
    LAMBDA = 1
    BIGNUMBER = 5
    MUTE = 1

##    timeF = open("data\SW22SWvsEffEFF.txt", 'a+')
##    timeF.seek(0)    

##    BEGINTIME = time.clock()
    print("#DRIVERS",D)
    ACNT = 0
    BCNT = 0
    CCNT = 0
    DCNT = 0
    ECNT = 0
        
    for i in range(D):
        AIRPORT = (int(rd.uniform(0,0.1)*PRECISION),int(rd.uniform(0.9,1)*PRECISION))
        ori = AIRPORT
        des = AIRPORT

        randPart = rd.uniform(0,1)


        dWORK = (int(rd.uniform(0.0,0.1)*PRECISION),int(rd.uniform(0.0,0.1)*PRECISION))
        dp = 0.2
        aWORK = (int(rd.uniform(0.7,1.0)*PRECISION),int(rd.uniform(0.7,1.0)*PRECISION))
        pa = 0.3
        bWORK = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        pb = 0.2
        cWORK = (int(rd.uniform(0.8,1.0)*PRECISION),int(rd.uniform(0.0,0.2)*PRECISION))
        pc = 0.2


        randArrive = rd.uniform(0,1)
        if randArrive <= 0.1:
            flightTime = T//5
        elif randArrive <= 0.2:
            flightTime = 2*T//5
        elif randArrive <= 0.3:
            flightTime = 3*T//5
        elif randArrive <= 0.5:
            flightTime = 4*T//5
        elif randArrive <= 0.6:
            flightTime = 2*T//5
        elif randArrive <= 0.7:
            flightTime = 3*T//5
        elif randArrive <= 0.8:
            flightTime = 4*T//5
        elif randArrive <= 1:
            flightTime = 5*T//5
            
        

        DCDEV = 1
        DCDET = 6
        MAXDEV = 30


        # 40% GO TO dWORK
        if randPart <= dp:
            if randArrive >= 0.5:
                ori = dWORK

                tim = int(rd.uniform(0,max(0,flightTime-Distance(ori,des))))
                delta = int(rd.uniform(0,0.2)*PRECISION)
                etim = flightTime-delta
                ptim = int(rd.uniform(0,max(0,(flightTime-Distance(ori,des))//2)))
            else:
                des = dWORK
                tim = flightTime
                delta = int(rd.uniform(0,0.7)*PRECISION)
                etim = min(max(flightTime+Distance(ori,des),1),T-1)+delta
                ptim = flightTime
            
            cdev = DCDEV
            cdet = DCDET
            cap = CAP
            val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
            rho = rhoo
            ACNT+=1
        # 20 % GO TO aWWORK
        elif randPart <= dp+pa:
            if randArrive >= 0.5:
                ori = aWORK

                tim = int(rd.uniform(0,max(0,flightTime-Distance(ori,des))))
                delta = int(rd.uniform(0,0.2)*PRECISION)
                etim = flightTime-delta
                ptim = int(rd.uniform(0,max(0,(flightTime-Distance(ori,des))//2)))
            else:
                des = aWORK
                tim = flightTime
                delta = int(rd.uniform(0,0.7)*PRECISION)
                etim = min(max(flightTime+Distance(ori,des),1),T-1)+delta
                ptim = flightTime
            
            cdev = DCDEV
            cdet = DCDET
            cap = CAP
            val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
            rho = rhoo
            maxdev = MAXDEV
            BCNT+=1
        # 10% GO TO SCHOOL
        elif randPart <= dp+pa+pb:
            if randArrive >= 0.5:
                ori = bWORK

                tim = int(rd.uniform(0,max(0,flightTime-Distance(ori,des))))
                delta = int(rd.uniform(0,0.2)*PRECISION)
                etim = flightTime-delta
                ptim = int(rd.uniform(0,max(0,(flightTime-Distance(ori,des))//2)))
            else:
                des = bWORK
                tim = flightTime
                delta = int(rd.uniform(0,0.7)*PRECISION)
                etim = min(max(flightTime+Distance(ori,des),1),T-1)+delta
                ptim = flightTime

                
            cdev = DCDEV
            cdet = DCDET
            cap = CAP
            val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
            rho = rhoo
            maxdev = MAXDEV
            CCNT+=1

        elif randPart <= dp+pa+pb+pc:
            if randArrive >= 0.5:
                ori = cWORK

                tim = int(rd.uniform(0,max(0,flightTime-Distance(ori,des))))
                delta = int(rd.uniform(0,0.2)*PRECISION)
                etim = flightTime-delta
                ptim = int(rd.uniform(0,max(0,(flightTime-Distance(ori,des))//2)))
            else:
                des = cWORK
                tim = flightTime
                delta = int(rd.uniform(0,0.7)*PRECISION)
                etim = min(max(flightTime+Distance(ori,des),1),T-1)+delta
                ptim = flightTime
                
            cdev = DCDEV
            cdet = DCDET
            cap = CAP
            val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
            rho = rhoo
            maxdev = MAXDEV
            DCNT+=1
        # REST ARE RANDOM
        else:
            if randArrive >= 0.5:
                ori = (int(rd.uniform(0,1.0)*PRECISION),int(rd.uniform(0.0,1.0)*PRECISION))

                tim = int(rd.uniform(0,max(0,flightTime-Distance(ori,des))))
                delta = int(rd.uniform(0,0.2)*PRECISION)
                etim = flightTime-delta
                ptim = int(rd.uniform(0,max(0,(flightTime-Distance(ori,des))//2)))
            else:
                des = (int(rd.uniform(0,1.0)*PRECISION),int(rd.uniform(0.0,1.0)*PRECISION))
                tim = flightTime
                delta = int(rd.uniform(0,0.7)*PRECISION)
                etim = min(max(flightTime+Distance(ori,des),1),T-1)+delta
                ptim = flightTime

            
            cdev = DCDEV
            cdet = DCDET
            cap = CAP
            val = int(Distance(ori,des)*cdet*(1.25+rd.uniform(0,1)))
            rho = rhoo
            maxdev = MAXDEV
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
        AIRPORT = (int(rd.uniform(0,0.1)*PRECISION),int(rd.uniform(0.9,1)*PRECISION))
        ori = AIRPORT
        des = AIRPORT

        randPart = rd.uniform(0,1)
        

        dWORK = (int(rd.uniform(0.0,0.1)*PRECISION),int(rd.uniform(0.0,0.1)*PRECISION))
        dp = 0.2
        aWORK = (int(rd.uniform(0.7,1.0)*PRECISION),int(rd.uniform(0.7,1.0)*PRECISION))
        pa = 0.3
        bWORK = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
        pb = 0.2
        cWORK = (int(rd.uniform(0.8,1.0)*PRECISION),int(rd.uniform(0.0,0.2)*PRECISION))
        pc = 0.2


        randArrive = rd.uniform(0,1)
        if randArrive <= 0.1:
            flightTime = T//5
        elif randArrive <= 0.2:
            flightTime = 2*T//5
        elif randArrive <= 0.3:
            flightTime = 3*T//5
        elif randArrive <= 0.5:
            flightTime = 4*T//5
        elif randArrive <= 0.6:
            flightTime = 2*T//5
        elif randArrive <= 0.7:
            flightTime = 3*T//5
        elif randArrive <= 0.8:
            flightTime = 4*T//5
        elif randArrive <= 1:
            flightTime = 5*T//5
            

        RCDEV = 1
        RCDET = 3
        MAXDEV = 30
        
        # 40% GO TO dWORK
        if randPart <= dp:
            neighborHood = dWORK
            if randArrive >= 0.5:
                ori = neighborHood

                tim = int(rd.uniform(0,max(0,flightTime-Distance(ori,des))))
                delta = int(rd.uniform(0,0.2)*PRECISION)
                etim = flightTime-delta
                ptim = int(rd.uniform(0,max(0,(flightTime-Distance(ori,des))//2)))
            else:
                des = neighborHood
                tim = flightTime
                delta = int(rd.uniform(0,0.7)*PRECISION)
                etim = min(max(flightTime+Distance(ori,des),1),T-1)+delta
                ptim = flightTime
                
            cdev = RCDEV
            cdet = RCDET
            val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
            lamb = val
            maxdev = MAXDEV
            ACNT +=1
            
        # 20 % GO TO aWork
        elif randPart <= dp+pa:
            neighborHood = aWORK
            if randArrive >= 0.5:
                ori = neighborHood

                tim = int(rd.uniform(0,max(0,flightTime-Distance(ori,des))))
                delta = int(rd.uniform(0,0.2)*PRECISION)
                etim = flightTime-delta
                ptim = int(rd.uniform(0,max(0,(flightTime-Distance(ori,des))//2)))
            else:
                des = neighborHood
                time = flightTime
                delta = int(rd.uniform(0,0.7)*PRECISION)
                etim = min(max(flightTime+Distance(ori,des),1),T-1)+delta
                ptim = flightTime
            cdev = RCDEV
            cdet = RCDET
            val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
            lamb = val
            maxdev = MAXDEV
            BCNT +=1
            
        # 10% GO TO bWORK
        elif randPart <= dp+pa+pb:
            neighborHood = bWORK
            if randArrive >= 0.5:
                ori = neighborHood

                tim = int(rd.uniform(0,max(0,flightTime-Distance(ori,des))))
                delta = int(rd.uniform(0,0.2)*PRECISION)
                etim = flightTime-delta
                ptim = int(rd.uniform(0,max(0,(flightTime-Distance(ori,des))//2)))
            else:
                des = neighborHood
                tim = flightTime
                delta = int(rd.uniform(0,0.7)*PRECISION)
                etim = min(max(flightTime+Distance(ori,des),1),T-1)+delta
                ptim = flightTime
            cdev = RCDEV
            cdet = RCDET
            val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
            lamb = val
            maxdev = MAXDEV
            CCNT +=1

        elif randPart <= dp+pa+pb+pc:
            neighborHood = cWORK
            if randArrive >= 0.5:
                ori = neighborHood

                tim = int(rd.uniform(0,max(0,flightTime-Distance(ori,des))))
                delta = int(rd.uniform(0,0.2)*PRECISION)
                etim = flightTime-delta
                ptim = int(rd.uniform(0,max(0,(flightTime-Distance(ori,des))//2)))
            else:
                des = neighborHood
                tim = flightTime
                delta = int(rd.uniform(0,0.7)*PRECISION)
                etim = min(max(flightTime+Distance(ori,des),1),T-1)+delta
                ptim = flightTime
            cdev = RCDEV
            cdet = RCDET
            val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
            lamb = val
            maxdev = MAXDEV
            DCNT +=1
            
        # REST ARE RANDOM
        else:
            neighborHood = (int(rd.uniform(0,1.0)*PRECISION),int(rd.uniform(0.0,1.0)*PRECISION))
            if randArrive >= 0.5:
                ori = neighborHood

                tim = int(rd.uniform(0,max(0,flightTime-Distance(ori,des))))
                delta = int(rd.uniform(0,0.2)*PRECISION)
                etim = flightTime-delta
                ptim = int(rd.uniform(0,max(0,(flightTime-Distance(ori,des))//2)))
            else:
                des = neighborHood
                tim = flightTime
                delta = int(rd.uniform(0,0.7)*PRECISION)
                etim = min(max(flightTime+Distance(ori,des),1),T-1)+delta
                ptim = flightTime
            cdev = RCDEV
            cdet = RCDET
            val = int(Distance(ori,des)*cdet*(1.5+rd.uniform(0,1)))
            lamb = val
            maxdev = MAXDEV
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

timeF = open("data\ScalabilityAirport.txt", 'a+')
timeF.seek(0)
newRho = 1.2
for D in range(1,101):
    R = 3*D
    for REPEAT in range(3):
        valSW_SW,valEFF_SW, valSW_EFF,valEFF_EFF, runtime, runtime2 = main(R, D, rhoo=newRho)
        timeF.write("%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" %(D, R,newRho,valSW_SW,valEFF_SW,valSW_EFF,valEFF_EFF,runtime,runtime2))
        timeF.seek(0)
timeF.close()

##DD = 2
##n = 4
##val, runtime = main(n,DD)
    
