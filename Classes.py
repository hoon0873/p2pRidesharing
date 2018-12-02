import numpy as np
import scipy.sparse

class Driver(object):
    def __init__(self, origin, destin, etime, ltime, ptime, cap, cdev = 1, cdet = 3,val=100, rho=0,maxDev=2e9):
        self.ori = np.array(origin)
        self.des = np.array(destin)
        self.et = etime
        self.lt = ltime
        self.pt = ptime
        self.cdev = cdev
        self.cdet = cdet
        self.val = val
        self.cap = cap
        self.rho = rho
        self.maxDev = maxDev

    def __str__(self):
        return "Origin: "+str(self.ori)+"\nDestin: "+str(self.des)+"\nWindow,PT: "+str([self.et,self.lt,self.pt])+'\n'+\
               "cap: "+str(self.cap)+" cdev: "+str(self.cdev)+" cdet: "+str(self.cdet)+" val: "+str(self.val)+" rho "+str(self.rho)+'\n'


class Passenger(object):
    def __init__(self, origin, destin, etime, ltime,ptime, cdev = 1, cdet = 3, val =100, lamb=100,maxDev=2e9):
        self.ori = np.array(origin)
        self.des = np.array(destin)
        self.et = etime
        self.lt = ltime
        self.pt = ptime
        self.cdev = cdev
        self.cdet = cdet
        self.val = val
        self.lamb = lamb
        self.maxDev = maxDev

    def __str__(self):
        return "Origin: "+str(self.ori)+"\nDestin: "+str(self.des)+"\nWindow: "+str([self.et,self.lt,self.pt])+'\n'\
               "cdev: "+str(self.cdev)+" cdet: "+str(self.cdet)+" val: "+str(self.val)+" lambda: "+str(self.lamb)+'\n'


class Matching(object):
    def __init__(self, arrSet, nRequests=None):
        """
        :param arrSet: Array (one element per driver) of sets (containing indices of passengers)
        """
        self.arrSet = arrSet
        self.nDrivers = len(self.arrSet)
        if nRequests is None:
            nRequests = self.getMaxRiderID(arrSet)
        self.nRequests = nRequests

        self.M = self.convertToMatrix(self.arrSet)
        if self.checkIfOverlap(self.M):
            raise Exception('Some rider is being matched to more than one driver!%s'%self.M)

    def convertToMatrix(self, arrSet):
        """
        Convert array of sets to sparse binary driver-rider matrix
        :return: M, a sparse binary matrix
        """
        sp = scipy.sparse.dok_matrix((self.nDrivers, self.nRequests), dtype=np.int)
        for d_id, d in enumerate(arrSet):
            for k in d:
                for r in k:
                    print(d_id, r)
                    sp[d_id, r] = 1
        return sp

    def checkIfOverlap(self, M):
        """
        :param M: sparse matrix driver-rider matrices
        :return: True if some rider is matched to more than one matrix
        """
        return np.any(M.sum(axis=0) > 1)

    def getMaxRiderID(self, arrSet):
        """
        Try to infer maximum rider ID from the keys in elemnts of arrSet.
        May not be accurate when there are riders which are not in the keys
        :param arrSet:
        :return: inferred highest id of rider
        """
        ret = -1
        for d in arrSet:
            for k in d:
                if len(k) <= 0:
                    continue
                ret = max(ret, max(k))
        return ret

    def __str__(self):
        return str(self.arrSet)
