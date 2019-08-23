'''
Solve LP *relaxation* of bin packing problem using column generation
assume that item_sizes are integers
'''

import numpy as np
import gurobipy as grb
from refactor import Solver
from copy import deepcopy
import random
import matplotlib as mpl
import matplotlib.pyplot as plt
from NewFeasibleBruthWTime import Distance

EPSILON = 10.**-10


class RandomizedPolicy(object):

    def __init__(self):
        self.drivers, self.requests = None, None
        self.solver = Solver()
        self.STABLE = False
        self.infCost = 0

    def setData(self, drivers, requests):
        self.drivers, self.requests = drivers, requests
        self.solver.setData(drivers, requests)

    def setSTABLE(self, STABLE):
        self.STABLE = STABLE

    def preprocess(self):
        STABLE = self.STABLE
        self.solver.preprocess(STABLE)

    def matchingToVector(self, matching):
        """ Adapter converting a list of dictionaries to a list of binary vectors """

    def solve(self, theta=None):
        """
        :param theta: threshold probability for each rider
        :return:
        """

        STABLE = self.STABLE

        nDrivers, nRequests = len(self.drivers), len(self.requests)
        self.checkSolutionExists()

        model = grb.Model("master")
        model.setParam('OutputFlag', 0)
##        model.setObjective(grb.GRB.MINIMIZE)
        costs, rtv, schedules = self.solver.COST, self.solver.RTV, self.solver.SCHEDULE

##        for d in range(nDrivers):
##            print("Driver "+str(d))
##            print(schedules[d])

        if theta is None:
            theta = [1./nRequests] * nRequests # TODO: refactor, change to something less conservative
        theta = self.preprocessTheta(theta, nRequests, schedules)
        
        self.infCost = self.ComputeInfCost(nRequests,schedules)
        print("INFCOST!!!!")
        print(self.infCost)

        #for z in range(len(theta)):
        #    theta[z] *= theta_ratio

        # Initialize set of basic matchings #
        # CK: hacky preprocessing to include at least one matching per rider
        # We want *a* feasible solution.
        # To achieve this, we add in, for each rider with positive theta, some matching containing it.
        # In the following, we greedily add matchings, each with just single driver matched to some rider r
        # NOTE: this does *not* guarantee a feasible solution unless theta is sufficiently small
        minimal_raw_matchings = self.getFeasibleBasis(nRequests, nDrivers, schedules, theta)

        # add random matchings
##        for ct in range(50):
##            minimal_raw_matchings.append(self.getRandomMatching(nRequests,nDrivers,schedules,theta,rtv))
        """
        print(minimal  _raw_matchings)
        for j in minimal_raw_matchings:
            print(j)
        assert False
        """
        matchings = [] # contains full matching info. #TODO refactor to class
        # matching_vectors = [] #
        matching_costs = []
        matching_riders = []
        for raw_matching in minimal_raw_matchings:
            riders_bin_vector, drivers_riders_bin_matrix, riders, cost = self.adaptMatching(raw_matching)
            matchings.append((riders_bin_vector, drivers_riders_bin_matrix, riders, cost))
            matching_riders.append(riders)
            matching_costs.append(cost)

        # Generate sum-to-one constraints
        constr_sum_to_one = model.addConstr(lhs=0,
                                            sense=grb.GRB.EQUAL,
                                            rhs=1.,
                                            name='constraint: probs ')

        # Generate list of constraints (one per rider)
        constr_rider_list = []
        for i in range(nRequests):
            constr_rider_list.append(model.addConstr(lhs=0,
                                                     sense=grb.GRB.GREATER_EQUAL,
                                                     rhs=theta[i],
                                                     name='constraint: item ' + str(i)))

        # Generate variables (non-negative constraints and through columns)
        var_list = []
        for idx in range(len(matching_riders)):
            temp_col = grb.Column(coeffs=[1.0]*(len(matching_riders[idx])+1),
                                  constrs=[constr_rider_list[k] for k in matching_riders[idx]] + [constr_sum_to_one])
            var_list.append(model.addVar(vtype='C',
                                         name='matching%d' % len(var_list),
                                         # name='pattern: ' + str(p),
                                         obj=matching_costs[idx],
                                         lb=0.,
                                         column=temp_col))

        objective_history = []
        riders_history = []
        solution_vectors = []
        rndCnt = 1
        iter_num = 0
##        model.setParam('OutputFlag', 0)
        while True:
            if rndCnt >= 100: break
            # Solve SmallPrimal
            model.optimize()
            if model.status != grb.GRB.OPTIMAL:
                rndCnt+=1
                raw_matching = self.getRandomMatching(nRequests,nDrivers,schedules,theta,rtv)
                riders_bin_vector, drivers_riders_bin_matrix, riders, cost = self.adaptMatching(raw_matching)
##                matchings.append((riders_bin_vector, drivers_riders_bin_matrix, riders, cost))
##                matching_riders.append(riders)
##                matching_costs.append(cost)

                
                
            else:
                objective_history.append(model.getAttr('ObjVal'))
                solution_vectors.append(model.getAttr('X'))
                # print_feedback(m, constr_list, num_items)

                # Save primal and dual variables
                """
                sav_primal = model.getAttr('x')
                sav_dual = model.getAttr('pi')
                sav_vbasis = model.getAttr('VBasis')
                sav_cbasis = model.getAttr('CBasis')
                """

                # Get a new, better matching by solving subproblem
                # duals = model.getAttr('pi')[1:]
                duals = [constr_rider_list[i].getAttr('Pi') for i in range(nRequests)]
                raw_matching = self.getImprovedMatching(duals)
                
                riders_bin_vector, drivers_riders_bin_matrix, riders, cost = self.adaptMatching(raw_matching)
                if riders in riders_history:
                    #print('Termination criterion met')
                    if iter_num > 5: break
                    print('But iter num too low %d' % iter_num)
##            print("RAW MATCHING TESTING: ")
##            print(raw_matching)
            matchings.append((riders_bin_vector, drivers_riders_bin_matrix, riders, cost))
            matching_riders.append(riders)
            matching_costs.append(cost)

            riders_history.append(riders)

            # Add column
            temp_col = grb.Column(coeffs=[1.0]*(len(matching_riders[-1])+1),
                                  constrs=[constr_rider_list[k] for k in matching_riders[-1]] + [constr_sum_to_one])
            var_list.append(model.addVar(vtype='C',
                                         name='matching%d' % len(var_list),
                                         # name='pattern: ' + str(p),
                                         obj=matching_costs[-1],
                                         lb=0.,
                                         column=temp_col))
            """
            temp_col = grb.Column(coeffs=[1.0]*sum(new_col),
                                  constrs=[constr_list[i] for i in range(num_items) if new_col[i]==1])
            var_list.append(model.addVar(vtype='C',
                                         name='pattern%d' % len(var_list),
                                         obj=1.,
                                         lb=0.,
                                         column=temp_col))
            """

            """
            # Does not seeem to help, or warm start already usd
            for i in range(len(var_list)-1):
                var_list[i].setAttr('PStart', sav_primal[i])
            var_list[-1].setAttr('PStart', 0.)

            for i, constr in enumerate(constr_list):
                constr.setAttr('DStart', sav_dual[i])
            """

            """
            # Does not seeem to help, or warm start already usd
            for i in range(len(var_list)-1):
                var_list[i].setAttr('VBasis', sav_vbasis[i])
            var_list[-1].setAttr('VBasis', 0)

            for i, constr in enumerate(constr_list):
                constr.setAttr('CBasis', sav_cbasis[i])
            """

            iter_num += 1
            #print('iteration: %d'%iter_num)
            if iter_num > 1000: break

        print('obj_hist')
        print(objective_history)
##        print('riders hist')
##        for j in riders_history:
##            print(j)
##        print('matching_costs')
##        print(matching_costs)

        print('Extracting solutions')
        if len(objective_history) != 0:
            final_objective = objective_history[-1]
            final_solution = solution_vectors[-1]
            final_costs = matching_costs
            final_matchings = matching_riders

        else:
            final_objective = None
            final_solution = None
            final_costs = None
            final_matchings = None

        return final_objective, final_solution, final_costs, final_matchings




    def preprocessTheta(self, THETA_DEFAULT, nRequests, schedules):
        # Change theta for infeasible riders to be 0
        # TODO: hoon says we should look at RV graph. Nothing works.
        theta = deepcopy(THETA_DEFAULT)
        print(theta)
        # theta = [THETA_DEFAULT] * nRequests
        feasible_riders = [False] * nRequests
        for s in schedules:
            for k, v in s.items():  # keys contain id's (tuple of integers) of riders
                if k[0] == 'E': continue # TODO: change this hacky logic...
                for j in k:
                    feasible_riders[j] = True
        # Set thetas of infeasible riders to 0.
        for j, z in enumerate(feasible_riders):
            if not z:
                theta[j] = 0.
        return theta

    def ComputeInfCost(self,nRequests,schedules):
        infCost = 0
        feasible_riders = [False] * nRequests
        for s in schedules:
            for k, v in s.items():  # keys contain id's (tuple of integers) of riders
                if k[0] == 'E': continue # TODO: change this hacky logic...
                for j in k:
                    feasible_riders[j] = True
        for j, z in enumerate(feasible_riders):
            if not z:
                infCost += self.requests[j].lamb
        return infCost
                

    def getFeasibleBasis(self, nRequests, nDrivers, schedules, theta):
        model = grb.Model("ThetaMaster")
        model.setParam('OutputFlag', 0)
##        model.setObjective(grb.GRB.MINIMIZE)
        costs, rtv, schedules = self.solver.COST, self.solver.RTV, self.solver.SCHEDULE
        
        tempRet,x,_,_,_ = self.solver.solve(STABLE = self.STABLE)
        raw_matching = x
        minimal_raw_matchings = [raw_matching]

        matchings = [] 
        matching_costs = []
        matching_riders = []
        raw_matchings_list = []
        for raw_matching in minimal_raw_matchings:
            riders_bin_vector, drivers_riders_bin_matrix, riders, cost = self.adaptMatching(raw_matching)
            matchings.append((riders_bin_vector, drivers_riders_bin_matrix, riders, cost))
            matching_riders.append(riders)
            matching_costs.append(cost)
            raw_matchings_list.append(raw_matching)

        # Generate sum-to-one constraints
        constr_sum_to_one = model.addConstr(lhs=0,
                                            sense=grb.GRB.EQUAL,
                                            rhs=1.,
                                            name='constraint: probs ')

        # Generate list of constraints (one per rider)
        constr_rider_list = []
        feasible_riders = [False] * nRequests
        for s in schedules:
            for k, v in s.items():  # keys contain id's (tuple of integers) of riders
                if k[0] == 'E': continue # TODO: change this hacky logic...
                for j in k:
                    feasible_riders[j] = True
        for i in range(nRequests):
            if feasible_riders[i] == True:
                constr_rider_list.append(model.addConstr(lhs=0,
                                                         sense=grb.GRB.GREATER_EQUAL,
                                                         rhs=0,
                                                         name='constraint: item ' + str(i)))
            else:
##                constr_rider_list.append(None)
                constr_rider_list.append(model.addConstr(lhs=0,
                                                         sense=grb.GRB.GREATER_EQUAL,
                                                         rhs=-1,
                                                         name='constraint: item ' + str(i)))

        # Generate variables (non-negative constraints and through columns)
        var_list = []
        temp_col = grb.Column(coeffs=[-1.0]*nRequests,
                              constrs=[constr_rider_list[k] for k in range(nRequests)])
        var_list.append(model.addVar(vtype='C',
                                     name='theta',
                                     obj = -1.,
                                     lb = 0.,
                                     column = temp_col))
        for idx in range(len(matching_riders)):
            temp_col = grb.Column(coeffs=[1.0]*len(matching_riders[idx])+[1.0],
                                  constrs=[constr_rider_list[k] for k in matching_riders[idx]] + [constr_sum_to_one])
            var_list.append(model.addVar(vtype='C',
                                         name='matching%d' % len(var_list),
                                         # name='pattern: ' + str(p),
                                         obj=0.,
                                         lb=0.,
                                         column=temp_col))

        objective_history = []
        riders_history = []
        solution_vectors = []
        thetaList = []
        rndCnt = 0
        iter_num = 0
##        model.setParam('OutputFlag', 0)
        while True:
            if rndCnt >= 100: break
            # Solve SmallPrimal
            model.optimize()
            if model.status != grb.GRB.OPTIMAL:
                modifiedCost = deepcopy(self.solver.COST)
                for d in modifiedCost:
                    for s, c in d.items():
                        if s[0] == 'E': continue
                        d[s] = random.randint(0,10)
                tempRet,x,_,_,_ = self.solver.solve(STABLE = self.STABLE,LAMB=None)
                raw_matching = x
                rndCnt+=1
                riders_bin_vector, drivers_riders_bin_matrix, riders, cost = self.adaptMatching(raw_matching)              
                
            else:
                objective_history.append(model.getAttr('ObjVal'))
                solution_vectors.append(model.getAttr('X'))
                thetaList.append(model.getVarByName("theta").X)
                # print_feedback(m, constr_list, num_items)

                # Save primal and dual variables
                """
                sav_primal = model.getAttr('x')
                sav_dual = model.getAttr('pi')
                sav_vbasis = model.getAttr('VBasis')
                sav_cbasis = model.getAttr('CBasis')
                """

                # Get a new, better matching by solving subproblem
                # duals = model.getAttr('pi')[1:]
                duals = [constr_rider_list[i].getAttr('Pi') for i in range(nRequests)]

                modifiedCost = deepcopy(self.solver.COST)
                for d in modifiedCost:
                    for s, c in d.items():
                        x = 0
                        if s[0] == 'E': continue
                        for rider in s:
                            x -= duals[rider]
                        d[s] = x
                tempRet,x,_,_,_ = self.solver.solve(modifiedCost,self.STABLE,LAMB=None)
                raw_matching = x
                
                
                riders_bin_vector, drivers_riders_bin_matrix, riders, cost = self.adaptMatching(raw_matching)
                if riders in riders_history:
                    #print('Termination criterion met')
                    if iter_num > 5: break
                    print('But iter num too low %d' % iter_num)
            matchings.append((riders_bin_vector, drivers_riders_bin_matrix, riders, cost))
            matching_riders.append(riders)
            matching_costs.append(cost)
            raw_matchings_list.append(raw_matching)
            riders_history.append(riders)

            # Add column
            temp_col = grb.Column(coeffs=[1.0]*(len(matching_riders[-1])+1),
                                  constrs=[constr_rider_list[k] for k in matching_riders[-1]] + [constr_sum_to_one])
            var_list.append(model.addVar(vtype='C',
                                         name='matching%d' % len(var_list),
                                         # name='pattern: ' + str(p),
                                         obj=0.,
                                         lb=0.,
                                         column=temp_col))

            iter_num += 1
##            print('iteration: %d'%iter_num)
            if iter_num > 1000: break
            
##        print('basis feasible matching')
##        print(raw_matchings_list)
        return raw_matchings_list

    
        """
        TODO: CHANGE THIS WEIRD DEFINITION OF A MATCHING.
        """
        if self.STABLE != True:
            satisfied = [False if theta[r] > 0. else True for r in range(nRequests)]
            minimal_matchings = []
            # TODO: Hoon says we should look at RV graph instead
            print(schedules)
            for s_id, s in enumerate(schedules):
                for k, v in s.items():  # keys contain id's (tuple of integers) of riders
                    if k[0] == 'E': continue  # TODO: change this hacky logic...
                    if len(k) == 1 and not satisfied[k[0]]:
                        satisfied[k[0]] = True

                        """ Create a raw matching"""
                        mm = [{'EMPTY%d'%i : 1.} for i in range(nDrivers)]
                        mm[s_id] = {(k[0],) : 1.}
                        minimal_matchings.append(mm)
            if not all(satisfied):
                raise Exception('Expected to have a basis which covers all riders!')
            return minimal_matchings
        
##        elif self.STABLE == True:

            
            

    def getRandomMatching(self, nRequests, nDrivers, schedules, theta, rtv):
        if type(self.STABLE) != int and self.STABLE == False:
            dOrder = ['d'+str(i) for i in range(nDrivers)]
            random.shuffle(dOrder)

            riderTaken = set()

            mm = [{'EMPTY%d'%i: 1.} for i in range(nDrivers)]

            for d in dOrder:
                N = rtv.in_degree(d)
                while True:
                    S = random.choice(list(rtv.predecessors(d)))
                    if S[0] == 'E': setS = set()
                    else: setS = set(S)

                    if setS - riderTaken == setS: break
                    
                mm[int(d[1])] = {S:1.}
                for j in S: riderTaken.add(j)
                
    ##        print(mm)
            return mm
        
        else:
            modifiedCost = deepcopy(self.solver.COST)
            for d in modifiedCost:
                for s, c in d.items():
                    if s[0] == 'E': continue
                    d[s] = random.randint(0,100)
            tempRet,x,_,_,_ = self.solver.solve(modifiedCost, STABLE = self.STABLE)
            raw_matching = x
            return raw_matching
            
                

    def getImprovedMatching(self, duals):

        # Create a modified version of the matching problem
        modified_cost = deepcopy(self.solver.COST)
##        print('duals')
##        print(duals)
        for d in modified_cost:
            for s, c in d.items():
                x = c
                if s[0] == 'E': continue
                for rider in s:
                    x -= duals[rider]
                d[s] = x

##        print('modified_cost')
##        for j in modified_cost:
##            print(j)

        temp_ret,x,_,_,_ = self.solver.solve(modified_cost,self.STABLE)
        return x

    def adaptMatching(self, matching_dictionary):
        '''
        Converts dictionary format of matching into
            - a single binary vector (of length nRequests)
            - a binary form of size nDrivers * nRequests with columns summing to no greater than 1
            - a list of tuples, each of which contain the riders chosen
            - cost vector
        Note that the first quantity is obtained by summing up rows of the second.
        :param: matching_dictionary
        :return:
        '''
        nRequests, nDrivers = len(self.requests), len(self.drivers)
        drivers_riders_bin_matrix = [[0 for x in range(nRequests)] for y in range(nDrivers)]
        riders_bin_vector = [0 for y in range(nRequests)]
        riders_list = []
        cost = 0.
        for driver_idx in range(len(matching_dictionary)):
            if matching_dictionary[driver_idx] is None: continue
            for sch, isChosen in matching_dictionary[driver_idx].items():
                if not isChosen: continue
                if sch[0] == 'E':
                    if isChosen >= 1.0:
                        cost += self.solver.COST[driver_idx][sch]
                    continue
                if isinstance(isChosen, float) or isinstance(isChosen, int):
                    if isChosen == 0.: continue
                else:
                    if isChosen.getAttr('X') <= 0.: continue
                for z in sch:
                    drivers_riders_bin_matrix[driver_idx][z] = 1
                    riders_bin_vector[z] += 1
                    riders_list.append(z)
                cost += self.solver.COST[driver_idx][sch]

##        print('riderLsit')
##        print(riders_list)
        for r in range(nRequests):
            if r in riders_list: continue
            else: cost += self.solver.lamb[r]

        return tuple(riders_bin_vector), \
               drivers_riders_bin_matrix, \
               tuple(riders_list), \
               cost


    def checkSolutionExists(self):
        pass


    def maxTheta(self):
        model = grb.Model("ThetaMaster")
        model.setParam('OutputFlag', 0)
        nRequests = len(self.requests)
        nDrivers = len(self.drivers)
##        model.setObjective(grb.GRB.MINIMIZE)
        costs, rtv, schedules = self.solver.COST, self.solver.RTV, self.solver.SCHEDULE
        
        tempRet,x,_,_,_ = self.solver.solve(STABLE = self.STABLE)
        raw_matching = x
        minimal_raw_matchings = [raw_matching]

        matchings = [] 
        matching_costs = []
        matching_riders = []
        raw_matchings_list = []
        for raw_matching in minimal_raw_matchings:
            riders_bin_vector, drivers_riders_bin_matrix, riders, cost = self.adaptMatching(raw_matching)
            matchings.append((riders_bin_vector, drivers_riders_bin_matrix, riders, cost))
            matching_riders.append(riders)
            matching_costs.append(cost)
            raw_matchings_list.append(raw_matching)

        # Generate sum-to-one constraints
        constr_sum_to_one = model.addConstr(lhs=0,
                                            sense=grb.GRB.EQUAL,
                                            rhs=1.,
                                            name='constraint: probs ')

        # Generate list of constraints (one per rider)
        constr_rider_list = []
        feasible_riders = [False] * nRequests
        for s in schedules:
            for k, v in s.items():  # keys contain id's (tuple of integers) of riders
                if k[0] == 'E': continue # TODO: change this hacky logic...
                for j in k:
                    feasible_riders[j] = True
        
        for i in range(nRequests):    
            if feasible_riders[i] == True:
                constr_rider_list.append(model.addConstr(lhs=0,
                                                         sense=grb.GRB.GREATER_EQUAL,
                                                         rhs=0,
                                                         name='constraint: item ' + str(i)))
            else:
##                constr_rider_list.append(None)
                constr_rider_list.append(model.addConstr(lhs=0,
                                                         sense=grb.GRB.GREATER_EQUAL,
                                                         rhs=-10,
                                                         name='constraint: item ' + str(i)))

        # Generate variables (non-negative constraints and through columns)
        var_list = []
        temp_col = grb.Column(coeffs=[-1.0]*nRequests,
                              constrs=[constr_rider_list[k] for k in range(nRequests)])
        var_list.append(model.addVar(vtype='C',
                                     name='theta',
                                     obj = -1.,
                                     lb = 0.,
                                     column = temp_col))
        for idx in range(len(matching_riders)):
            temp_col = grb.Column(coeffs=[1.0]*len(matching_riders[idx])+[1.0],
                                  constrs=[constr_rider_list[k] for k in matching_riders[idx]] + [constr_sum_to_one])
            var_list.append(model.addVar(vtype='C',
                                         name='matching%d' % len(var_list),
                                         # name='pattern: ' + str(p),
                                         obj=0.,
                                         lb=0.,
                                         column=temp_col))

        objective_history = []
        riders_history = []
        solution_vectors = []
        thetaList = []
        rndCnt = 0
        iter_num = 0
##        model.setParam('OutputFlag', 0)
        while True:
            if rndCnt >= 100: break
            # Solve SmallPrimal
            model.optimize()
            if model.status != grb.GRB.OPTIMAL:
                modifiedCost = deepcopy(self.solver.COST)
                for d in modifiedCost:
                    for s, c in d.items():
                        if s[0] == 'E': continue
                        d[s] = random.randint(0,10)
                tempRet,x,_,_,_ = self.solver.solve(STABLE = self.STABLE)
                raw_matching = x
                rndCnt+=1
                riders_bin_vector, drivers_riders_bin_matrix, riders, cost = self.adaptMatching(raw_matching)              
                
            else:
                objective_history.append(model.getAttr('ObjVal'))
                solution_vectors.append(model.getAttr('X'))
                thetaList.append(model.getVarByName("theta").X)
                # print_feedback(m, constr_list, num_items)

                # Save primal and dual variables
                """
                sav_primal = model.getAttr('x')
                sav_dual = model.getAttr('pi')
                sav_vbasis = model.getAttr('VBasis')
                sav_cbasis = model.getAttr('CBasis')
                """

                # Get a new, better matching by solving subproblem
                # duals = model.getAttr('pi')[1:]
                duals = [constr_rider_list[i].getAttr('Pi') for i in range(nRequests)]

                modifiedCost = deepcopy(self.solver.COST)
                for d in modifiedCost:
                    for s, c in d.items():
                        x = 0
                        if s[0] == 'E': continue
                        for rider in s:
                            x -= duals[rider]
                        d[s] = x
                tempRet,x,_,_,_ = self.solver.solve(modifiedCost,self.STABLE,LAMB=None)
                raw_matching = x
                
                
                riders_bin_vector, drivers_riders_bin_matrix, riders, cost = self.adaptMatching(raw_matching)
                if riders in riders_history:
                    #print('Termination criterion met')
                    if iter_num > 5: break
                    print('But iter num too low %d' % iter_num)
##                print(duals)
##                print(raw_matching)
            matchings.append((riders_bin_vector, drivers_riders_bin_matrix, riders, cost))
            matching_riders.append(riders)
            matching_costs.append(cost)
            raw_matchings_list.append(raw_matching)
            riders_history.append(riders)

            # Add column
            temp_col = grb.Column(coeffs=[1.0]*(len(matching_riders[-1])+1),
                                  constrs=[constr_rider_list[k] for k in matching_riders[-1]] + [constr_sum_to_one])
            var_list.append(model.addVar(vtype='C',
                                         name='matching%d' % len(var_list),
                                         # name='pattern: ' + str(p),
                                         obj=0.,
                                         lb=0.,
                                         column=temp_col))

            iter_num += 1
            #print('iteration: %d'%iter_num)
            if iter_num > 1000: break
            
##        print('basis feasible matching')
##        print(raw_matchings_list)
##        print('obj vec')
##        print(objective_history)
##        print('soln vec')
##        print(solution_vectors[-1])
##        print('thetaList')
##        print(thetaList)
        return thetaList[-1]

    # Calculate the cost of not matching anyone
    def baseCost(self):
        COST = self.solver.COST
        nRequests, nDrivers = len(self.requests), len(self.drivers)
        retCost = 0

        for r in range(nRequests):
            retCost += self.requests[r].lamb

        for i in range(nDrivers):
            retCost += COST[i]['EMPTY'+str(i)]

        return retCost
        
        



if __name__ == '__main__':
##    from generate import DataGenerator
    from generate    import DataGenerator

##    nRequests = 90
##    nDrivers = 10

    nDrivers  = 1
    nRequests = 4

    seed = 142857
    print('Generating data using seed %d...'%seed)
    generator = DataGenerator(n=nRequests, DD=nDrivers, PRECISION=50, CAP=4, rhoo=0.8, seed=seed)
    drivers, requests = generator.generate()

    MAXDIST = Distance(drivers[0].ori, drivers[0].des)
    for rr in requests:
        if MAXDIST <= Distance(rr.ori,rr.des):
            MAXDIST = Distance(rr.ori,rr.des)
    for rr in drivers:
        if MAXDIST <= Distance(rr.ori,rr.des):
            MAXDIST = Distance(rr.ori,rr.des)
        

    policySolver = RandomizedPolicy()
    policySolver.setData(drivers, requests)
    policySolver.preprocess()

    random.seed(seed) 
    
    baseline_theta = policySolver.maxTheta()
    theta_basic = [baseline_theta for j in range(nRequests)]

    allObjs = []
    curMaxTheta = baseline_theta

##    stableRatio = [10*MAXDIST,5*MAXDIST,3*MAXDIST,2*MAXDIST,MAXDIST,0.5*MAXDIST]
    stableRatio = [5,10]    
    stableScale = 1

    
    StableList = [stableRatio[i]*stableScale for i in range(len(stableRatio))]
    StableList.append(True)
    StableList = [False] + StableList

##    infCost = policySolver.infCost(nRequests,schedules)
    infCost = policySolver.infCost

    for STBOOL in StableList:
##    for STBOOL in [False]:
        allObjs.append([])
        
        policySolver.setSTABLE(STBOOL)
        if STBOOL != False:
            policySolver.setSTABLE(STBOOL)
            curMaxTheta = policySolver.maxTheta()
            print("Im currnetyly printing out maxTheta: ",curMaxTheta, STBOOL)
##    ##    theta_basic = [random.uniform(baseline_theta*0.3, baseline_theta*1) for j in range(nRequests)]

        """
        final_objective, final_solution, final_costs, final_matchings = policySolver.solve(theta=theta_basic)
        print('----')
        for m in final_matchings: # bin matrix
            print(m)
        print('----')
        print(final_solution)
        print(final_costs)
        """

        objectives = []
        resultingTheta = []
        solutionList = []
        costList = []
        matchingList = []
        # change_idx = 8
        
        theta_ratios = [0.0, 0.1, 0.2,1/4, 0.3,1/3, 0.4, 0.5, 0.6,2/3, 0.7,3/4, 0.8, 0.9, 1.]
##        BIGNUMBER = 10
##        theta_ratios = [i/BIGNUMBER for i in range(BIGNUMBER+1)]
        
##        theta_ratios=[0.0,0.7]
        theta_scaled = deepcopy(theta_basic)
        for theta_ratio in theta_ratios:
            theta_scaled = [theta_basic[i] * theta_ratio for i in range(len(theta_basic))]
            
            if theta_scaled[0] > curMaxTheta:
                resultingTheta.append(theta_scaled)
                objectives.append(None)
                solutionList.append(None)
                costList.append(None)
                matchingList.append(None)
                continue
            #theta_scaled[change_idx] = theta_basic[change_idx] * theta_ratio
            print('\n\n\n')
            print('SOLVING: '+str(STBOOL)+'   '+str(theta_scaled[0])+'   '+str(curMaxTheta))
            print('-------------------------------------------------------')
            final_objective, final_solution, final_costs, final_matchings = policySolver.solve(theta = theta_scaled)
            resultingTheta.append(theta_scaled)
##            objectives.append(final_objective)
            objectives.append(final_objective-infCost)
            solutionList.append(final_solution)
            costList.append(final_costs)
            matchingList.append(final_matchings)
        thetaAxis = [theta_ratios[i]*baseline_theta for i in range(len(theta_ratios))]
        print('Theta:')
        print(thetaAxis)
        print('objectives:')
        print(objectives)
##        print([objectives[i+1]-objectives[i] for i in range(len(objectives)-1)])
    ##    print('solution:')
    ##    print(solutionList)
    ##    print('cost:')
    ##    print(costList)
    ##    print('matching:')
    ##    print(matchingList)

        """
        print(m.getAttr('x'))
        for i in range(5):
            print(x[i])
        print(objSW)
        """

        """
        # item_sizes = np.array([4, 8, 1, 4, 2, 1])
        # bin_size = 10
        import timeit

        import random
        random.seed(0)
        item_sizes = np.array([random.randint(0, 10) for x in range(500)])
        # item_sizes = np.array([4, 8, 1, 4, 2, 1] * 100)
        bin_size = 10

        def wrap():
            main(item_sizes, bin_size)

        print(timeit.timeit(wrap, number=1))
        """
        allObjs[-1] = objectives

    ## ADD Base cost
    BaseCost = policySolver.baseCost()
    allObjs.append([])
    allObjs[-1] = [BaseCost-infCost]*len(thetaAxis)
##    allObjs[-1] = [BaseCost]*len(thetaAxis)
    StableList.append("Base Cost")
    
    for i in range(len(StableList)):
        if StableList[i] == False and type(StableList[i]) != int:
            StableList[i] = 'Not Stable'
        elif StableList[i] == True and type(StableList[i]) != int:
            StableList[i] = 'Stable'
        elif type(StableList[i]) == int:
            if StableList[i]//10 == StableList[i]/10:
                StableList[i] = str(StableList[i]//10)+'(Cost of a rider)'
            else:
                StableList[i] = str(StableList[i]/10)+'(Cost of a rider)'
            


    print('all objectives')
    print(thetaAxis)
    print(allObjs)



##### 90 Result
####    allSoln = [[12029.0, 12043.716666666664, 12058.433333333334, 12073.15, 12087.866666666669, 12102.583333333332, 12117.300000000001, 12132.016666666665, 12146.733333333335, 12165.099999999997, 12188.333333333332], [12029.4, 12044.116666666665, 12058.833333333332, 12073.55, 12088.266666666668, 12102.983333333334, 12117.699999999999, 12132.416666666664, 12147.133333333337, 12165.500000000002, 12188.733333333332], [12029.8, 12044.516666666665, 12059.233333333334, 12073.949999999995, 12088.666666666666, 12103.383333333331, 12118.099999999999, 12132.816666666664, 12147.533333333333], [12030.2], [12030.6], [12031.0], [12031.4, 12046.116666666663, 12060.833333333334, 12075.55, 12090.266666666668, 12104.983333333332, 12119.7, 12134.416666666664, 12149.133333333335, 12167.499999999996, 12190.733333333332], [12077.8], [12717.2, 12717.2, 12717.2, 12717.2, 12717.2, 12717.2, 12717.2, 12717.2, 12717.2, 12717.2, 12717.2]]
####    thetaAxis = [0.0, 0.016666666666666666, 0.03333333333333333, 0.049999999999999996, 0.06666666666666667, 0.08333333333333333, 0.09999999999999999, 0.11666666666666665, 0.13333333333333333, 0.15, 0.16666666666666666]



    allSoln = []
    EPSILONN = 0
    for ct in range(len(allObjs)):
        allSoln.append([])
        for j in range(len(allObjs[ct])):
            if allObjs[ct][j] != None:
                allSoln[ct].append(allObjs[ct][j]+ct*EPSILONN)

    plt.rcParams.update({'font.size':22})
    plt.figure()
    plt.xlabel(r'Minimum matched probability ($\theta$)')
    plt.ylabel('Total Cost')

    markers = ['1', 'v', 'h', '>', 's', '<', 'X', 'p', '.', '+', '^', 'd', 'P', 'o', '']

    for i in range(len(allObjs)):
        if StableList[i] == 'Base Cost':
            curMarker = ''
        else:
            curMarker = markers[i]
        plt.plot(thetaAxis[:len(allSoln[i])], allSoln[i], marker = curMarker,label=str(StableList[i]))
    
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
       ncol=1, mode="expand", borderaxespad=0.)
    plt.legend(loc=2, prop={'size':22})
    plt.savefig('figures/tradeOff0220'+str(nRequests)+'.pdf',bbox_inches='tight')

    print(policySolver.solver.COST)
