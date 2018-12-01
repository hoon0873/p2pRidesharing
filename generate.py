import time
import random as rd
from Classes import Driver
from Classes import Passenger
import NewMatchingRTV3 as nrtv3
from NewFeasibleBruthWTime import Distance
import logging

logger = logging.getLogger('DataGenerator')

class DataGenerator(object):
    '''
    TODO: add magic numbers to initialization
    '''
    def __init__(self, n=15, DD=5, PRECISION=30, CAP=4, rhoo=1.2, seed=142857):
        self.n = n
        self.DD = DD
        self.PRECISION = PRECISION
        self.CAP = CAP
        self.rhoo = rhoo

        rd.seed(seed)

    def generate(self):
        '''
        :return: tuple of (drivers, requests)
        '''

        # Handle aliases
        n, DD, PRECISION, CAP, rhoo = self.n, self.DD, self.PRECISION, self.CAP, self.rhoo
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
        logger.info("#DRIVERS",D)
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


            dWORK = (int(rd.uniform(0.0,0.1)*PRECISION),int(rd.uniform(0.0,0.1)*PRECISION))
            dp = 0.4
            aWORK = (int(rd.uniform(0.7,1.0)*PRECISION),int(rd.uniform(0.7,1.0)*PRECISION))
            pa = 0.2
            bWORK = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
            pb = 0.1
            cWORK = (int(rd.uniform(0.2,0.5)*PRECISION),int(rd.uniform(0.6,0.9)*PRECISION))
            pc = 0.2

            # TODO: use random.choice instead
            # TODO: refactor please
            randPart = rd.uniform(0,1)
            # 40% GO TO dWORK
            if randPart <= dp:
                des = dWORK

                tim = int(rd.uniform(0,max(T//10,1))) # EARLY WINDOW
                delta = int(rd.uniform(0,0.5)*PRECISION)
                etim = min(max(T//10,1)+Distance(ori,des),T-1) #STRICT WINDOW
                ptim = max(T//10,1) #STRICT PTIME

                # ADDED BY CK FOR SANITY
                # etim = min(max(T//3,1)+Distance(ori,des),T-1) #STRICT WINDOW
                # ptim = max(T//3,1) #STRICT PTIME

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

                # ADDED BY CK FOR SANITY
                # etim = min(max(T//2,1)+Distance(ori,des),T-1) #STRICT WINDOW
                # ptim = tim+delta//2 #less strict PTIME

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
            logger.info('drivers.append(Driver(',ori,",",des,",",tim,",",etim,","
                  ,ptim,",",cap,",",cdev,",",cdet,",",val,",",rho,",",maxdev,'))')
        logger.info('# COUNTS: ',ACNT,BCNT,CCNT,DCNT,ECNT)
        logger.info("#REQUESTS",R)

        ACNT = 0
        BCNT = 0
        CCNT = 0
        DCNT = 0
        ECNT = 0
        for i in range(R):
            NEIGHBORHOOD = (int(rd.uniform(0.45,0.55)*PRECISION),int(rd.uniform(0.45,0.55)*PRECISION))
            ori = NEIGHBORHOOD


            dWORK = (int(rd.uniform(0.0,0.3)*PRECISION),int(rd.uniform(0.0,0.3)*PRECISION))
            dp = 0.4
            aWORK = (int(rd.uniform(0.7,1.0)*PRECISION),int(rd.uniform(0.7,1.0)*PRECISION))
            pa = 0.2
            bWORK = (int(rd.uniform(0.4,0.7)*PRECISION),int(rd.uniform(0,0.2)*PRECISION))
            pb = 0.1
            cWORK = (int(rd.uniform(0.3,0.55)*PRECISION),int(rd.uniform(0.3,0.55)*PRECISION))
            pc = 0.2

            # TODO: use random.choice instead...
            # TODO: refactor please
            randPart = rd.uniform(0,1)
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
            logger.info('reqs.append(Passenger(',ori,",",des,",",tim,",",etim,",",ptim,
                  ",",cdev,",",cdet,",",val,",",lamb,",",maxdev,'))')
        logger.info('# COUNTS: ',ACNT,BCNT,CCNT,DCNT,ECNT)

        newRho = rhoo
        for i in range(D):
            drivers[i].rho = newRho

        return drivers, reqs


if __name__ == '__main__':
    generator = DataGenerator()

    print('Generating data...')
    drivers, requests = generator.generate()

    print('Solving rtv')
    m,x,valSW_SW,valEFF_SW,runtime = nrtv3.MatchingRTV(drivers,requests,RHO=None)
    print(valSW_SW,valEFF_SW)
