'''
Solve LP *relaxation* of bin packing problem using column generation
assume that item_sizes are integers
'''

import numpy as np
import gurobi as grb
from refactor import Solver
from copy import deepcopy
import random
import matplotlib as mpl
import matplotlib.pyplot as plt

EPSILON = 10.**-10


class RandomizedPolicy(object):

    def __init__(self):
        self.drivers, self.requests = None, None
        self.solver = Solver()

    def setData(self, drivers, requests):
        self.drivers, self.requests = drivers, requests
        self.solver.setData(drivers, requests)

    def preprocess(self):
        self.solver.preprocess()

    def matchingToVector(self, matching):
        """ Adapter converting a list of dictionaries to a list of binary vectors """

    def solve(self, theta=None):
        """
        :param theta: threshold probability for each rider
        :return:
        """

        nDrivers, nRequests = len(self.drivers), len(self.requests)
        self.checkSolutionExists()

        model = grb.Model("master")
        model.setParam('OutputFlag', 1)
        costs, rtv, schedules = self.solver.COST, self.solver.RTV, self.solver.SCHEDULE

        if theta is None:
            theta = [1./nRequests] * nRequests # TODO: refactor, change to something less conservative
        theta = self.preprocessTheta(theta, nRequests, schedules)

        #for z in range(len(theta)):
        #    theta[z] *= theta_ratio

        print('THETA:')
        print(theta)

        # Initialize set of basic matchings #
        # CK: hacky preprocessing to include at least one matching per rider
        # We want *a* feasible solution.
        # To achieve this, we add in, for each rider with positive theta, some matching containing it.
        # In the following, we greedily add matchings, each with just single driver matched to some rider r
        # NOTE: this does *not* guarantee a feasible solution unless theta is sufficiently small
        minimal_raw_matchings = self.getFeasibleBasis(nRequests, nDrivers, schedules, theta)
        """
        print(minimal_raw_matchings)
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

        iter_num = 0
        while True:
            # Solve SmallPrimal
            model.optimize()
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
                print('Termination criterion met')
                if iter_num > 5: break
                print('But iter num too low %d' % iter_num)
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
            print('iteration: %d'%iter_num)
            if iter_num > 1000: break

        print('obj_hist')
        print(objective_history)
        print('riders hist')
        for j in riders_history:
            print(j)
        print('matching_costs')
        print(matching_costs)

        print('Extracting solutions')
        final_objective = objective_history[-1]
        final_solution = solution_vectors[-1]
        final_costs = matching_costs[-1]
        final_matchings = matching_riders

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

    def getFeasibleBasis(self, nRequests, nDrivers, schedules, theta):
        """
        TODO: CHANGE THIS WEIRD DEFINITION OF A MATCHING.
        """
        satisfied = [False if theta[r] > 0. else True for r in range(nRequests)]
        minimal_matchings = []
        # TODO: Hoon says we should look at RV graph instead
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

    def getImprovedMatching(self, duals):

        # Create a modified version of the matching problem
        modified_cost = deepcopy(self.solver.COST)
        print('duals')
        print(duals)
        for d in modified_cost:
            for s, c in d.items():
                x = c
                if s[0] == 'E': continue
                for rider in s:
                    x -= duals[rider]
                d[s] = x

        print('modified_cost')
        for j in modified_cost:
            print(j)

        temp_ret,x,_,_,_ = self.solver.solve(modified_cost)
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
        print('WTF')
        for q in matching_dictionary:
            print(q)
        for driver_idx in range(len(matching_dictionary)):
            if matching_dictionary[driver_idx] is None: continue
            for sch, isChosen in matching_dictionary[driver_idx].items():
                if not isChosen: continue
                if sch[0] == 'E':
                    if isChosen >= 1.0:
                        cost += self.solver.COST[driver_idx][sch]
                    continue
                print(isChosen)
                if isinstance(isChosen, float) or isinstance(isChosen, int):
                    if isChosen == 0.: continue
                else:
                    if isChosen.getAttr('X') <= 0.: continue
                    print('isChosen', isChosen.getAttr('X'))
                for z in sch:
                    drivers_riders_bin_matrix[driver_idx][z] = 1
                    riders_bin_vector[z] += 1
                    riders_list.append(z)
                cost += self.solver.COST[driver_idx][sch]

        return tuple(riders_bin_vector), \
               drivers_riders_bin_matrix, \
               tuple(riders_list), \
               cost


    def checkSolutionExists(self):
        pass





if __name__ == '__main__':
    from generate import DataGenerator

    # nRequests = 100
    # nDrivers = 10

    nRequests = 10
    nDrivers = 10

    seed = 142857
    print('Generating data using seed %d...'%seed)
    generator = DataGenerator(n=nRequests, DD=nDrivers, PRECISION=50, CAP=4, rhoo=1.2, seed=seed)
    drivers, requests = generator.generate()

    policySolver = RandomizedPolicy()
    policySolver.setData(drivers, requests)
    policySolver.preprocess()

    print(policySolver.solver.RTV)

    random.seed(seed)
    baseline_theta = 1./ nRequests
    theta_basic = [random.uniform(baseline_theta*0.3, baseline_theta*1) for j in range(nRequests)]

    final_objective, final_solution, final_costs, final_matchings = policySolver.solve(theta=theta_basic)
    print('----')
    for m in final_matchings: # bin matrix
        print(m)
    print('----')
    print(final_solution)
    print(final_costs)


    objectives = []
    change_idx = 8
    theta_ratios = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.]
    theta_scaled = deepcopy(theta_basic)
    for theta_ratio in theta_ratios:
        theta_scaled[change_idx] = theta_basic[change_idx] * theta_ratio
        final_objective, final_solution, final_costs, final_matchings = policySolver.solve(theta = theta_scaled)
        objectives.append(final_objective)
        print(theta_scaled)
        assert False

    print('objectives:')
    print(objectives)
    print([objectives[i+1]-objectives[i] for i in range(len(objectives)-1)])


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
