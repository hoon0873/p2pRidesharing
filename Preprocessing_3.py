#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 15:47:38 2018

@author: wangweilong
"""

import numpy as np

def Preprocessing(inputs):

    A = inputs

    Subset = []
    deleteRow = []
    deleteCol = []

    row = A.shape[0]
    col = A.shape[1]

##    for i in range(row):
##        if A[i].sum() == 0:
####            deleteRow.append(i)
##    
##    for j in range(col):
##        if A[:,j].sum() == 0:
####            deleteCol.append(j)

            
    kk = 0;

    for i in range(row):
        index = []
        for j in range(col):
            if A[i][j] == 1:
                index.append(j)

        k = 0
        size = len(index)
        if size == 0: continue
        for ii in range(size):
            if A[:,index[ii]].sum() == 1:
                k += 1
        
        if k == size:
            rider = index
            driver = i
            Subset.append([driver, rider])
            kk += 1


    deleteRow += [Subset[i][0] for i in range(len(Subset))]
    setCol = set()
    for i in range(len(Subset)):
        for j in range(len(Subset[i][1])):
            setCol.add(Subset[i][1][j])
    
    deleteCol += list(setCol)
    A = np.delete(A, deleteRow, 0)
    A = np.delete(A, deleteCol, 1)

    return A, Subset, deleteRow, deleteCol


if __name__ == "__main__":
    B = np.array([[0,1,1,0,0,0,0,0],
                  [1,0,0,0,0,0,0,0],
                  [0,0,1,0,0,0,0,1],
                  [0,0,0,0,1,0,1,0],
                  [0,0,0,0,0,1,0,0],
                  [0,1,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0]])


    A, Subset, dR, dC = Preprocessing(B)

    N = 100
    C = np.zeros((N,N))
    for i in range(N):
        C[i][i] = 1

    D,S,ddR,ddC = Preprocessing(C)

