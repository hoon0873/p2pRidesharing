'''
Solves randomized matching problem with min. constraints for each rider to be fufilled.
Uses column generation.
'''

import numpy as np
import gurobi as grb
from ridesharing import RidesharingProblem, Matching
from copy import deepcopy
import random
import logging

logger = logging.getLogger('RandomizedPolicy')

class RandomizedPolicy(object):

    def __init__(self):
        self.drivers, self.requests = None, None
        self.problem = RidesharingProblem()

    def setData(self, drivers, requests):
        self.drivers, self.requests = drivers, requests
        self.problem.setData(drivers, requests)

    def preprocess(self):
        self.problem.preprocess()

    def solve(self, theta=None):
        """
        :param theta: threshold probability for each rider
        :return:
        """

        # Initialize aliases
        nDrivers, nRequests = len(self.drivers), len(self.requests)
        costs, rtv, schedules = self.problem.COST, self.problem.RTV, self.problem.SCHEDULE
        self.checkSolutionExists() # TODO: check if there exists a solution.

        # Construct model.
        model = grb.Model("master")
        model.setParam('OutputFlag', 1) # Suppress output from gurobi

        # Set theta to be a conservative value of it was not passed
        if theta is None:
            theta = [1./nRequests] * nRequests # TODO: change to something less conservative?

        # Remove theta's which cause problem to be infeasible
        theta = self.preprocessTheta(theta, nRequests, schedules)

        # Initialize set of basic matchings #
        minimal_matchings = self.getFeasibleBasis(nRequests, nDrivers, schedules, theta)
        matchings = [] # contains full matching info.
        matching_costs = []
        for matching in minimal_matchings:
            cost = self.computeMatchingCost(matching)
            matchings.append(matching)
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
        for idx in range(len(matchings)):
            matching, cost = matchings[idx], matching_costs[idx]
            temp_col = grb.Column(coeffs=[1.0]*(len(matching.riderSet)+1), # One additional constraint for probability simplex
                                  constrs=[constr_rider_list[k] for k in matching.riderSet] + [constr_sum_to_one])
            var_list.append(model.addVar(vtype='C',
                                         name='matching%d' % len(var_list),
                                         obj=cost,
                                         lb=0.,
                                         column=temp_col))

        objective_history = []
        solution_vectors = []

        iter_num = 0
        while True:
            # Solve primal solution with limited columns
            model.optimize()
            objective_history.append(model.getAttr('ObjVal'))
            solution_vectors.append(model.getAttr('X'))

            # Save primal and dual variables
            """
            sav_primal = model.getAttr('x')
            sav_dual = model.getAttr('pi')
            sav_vbasis = model.getAttr('VBasis')
            sav_cbasis = model.getAttr('CBasis')
            """

            # Get a new, better matching by solving subproblem
            duals = [constr_rider_list[i].getAttr('Pi') for i in range(nRequests)]
            new_matching = self.getImprovedMatching(duals)
            if new_matching in matchings:
                logger.info('Termination criterion met.')
                if iter_num > 5:
                    break
                logger.info('But iter num too low %d' % iter_num)
            matchings.append(new_matching)

            cost = self.computeMatchingCost(new_matching)
            matching_costs.append(cost)

            # Add column
            temp_col = grb.Column(coeffs=[1.0]*(len(matchings[-1].riderSet)+1),
                                  constrs=[constr_rider_list[k] for k in matchings[-1].riderSet] + [constr_sum_to_one])
            var_list.append(model.addVar(vtype='C',
                                         name='matching%d' % len(var_list),
                                         # name='pattern: ' + str(p),
                                         obj=matching_costs[-1],
                                         lb=0.,
                                         column=temp_col))

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
            logger.info('iteration: %d'%iter_num)
            if iter_num > 1000: break

        """
        print('obj_hist')
        print(objective_history)
        print('riders hist')
        for j in matchings:
            print(j)
        print('matching_costs')
        print(matching_costs)

        print('Extracting solutions')
        """
        final_objective = objective_history[-1]
        final_solution = solution_vectors[-1]
        final_costs = matching_costs
        final_matchings = matchings

        return final_objective, final_solution, final_costs, final_matchings

    def preprocessTheta(self, theta, nRequests, schedules):
        """
        Change theta for infeasible riders to be 0
        :param theta: input theta
        :param nRequests:
        :param schedules: preprocessed schedules.
        :return: theta - modified list of theta
        """
        # TODO: hoon says we should look at RV graph. Nothing works.
        theta = deepcopy(theta)
        feasible_riders = [False] * nRequests
        for s in schedules:
            for k, v in s.items():  # keys contain id's (tuple of integers) of riders
                if len(k) == 0: continue # Empty set
                for j in k:
                    feasible_riders[j] = True
        # Set thetas of infeasible riders to 0.
        for j, z in enumerate(feasible_riders):
            if not z:
                theta[j] = 0.
        return theta

    def getFeasibleBasis(self, nRequests, nDrivers, schedules, theta):
        """
        Hacky preprocessing to include at least one matching per rider. We want *a* feasible solution.
        To achieve this, we add in, for each rider with positive theta, some matching containing it.
        In the following, we greedily add matchings (per rider),
        each with just single driver matched to some rider r.

        NOTE: this does *not* guarantee a feasible solution unless theta is sufficiently small
        :return list of matching objects.
        """
        satisfied = [False if theta[r] > 0. else True for r in range(nRequests)]
        minimal_matchings = []
        # TODO: Hoon says we should look at RV graph instead
        for s_id, s in enumerate(schedules):
            for k, v in s.items():  # keys contain id's (tuple of integers) of riders
                if len(k) == 0: continue  # empty set
                if len(k) == 1 and not satisfied[k[0]]:
                    satisfied[k[0]] = True

                    """ Create a matching and store it """
                    mm = [set() for i in range(nDrivers)]
                    mm[s_id] = set([k[0]])
                    mm = Matching(mm, nRequests)
                    minimal_matchings.append(mm)

        if not all(satisfied):
            raise Exception('Expected to have a basis which covers all riders!')
        return minimal_matchings

    def getImprovedMatching(self, duals):
        """
        Compute an improved matching for column generation
        :param duals: vector containing dual variables.
        :return: a Matching object containing the proposed matching
        """
        # Create a modified version of the matching problem
        modified_cost = deepcopy(self.problem.COST)
        for d in modified_cost:
            for s, c in d.items():
                x = c
                if len(s) == 0: continue
                for rider in s:
                    x -= duals[rider]
                d[s] = x

        x, _, _, _ = self.problem.solve(modified_cost)
        return x

    def computeMatchingCost(self, matching):
        """
        :param matching: Matching object
        :return: float containing the cost of this matching (under default costs)
        """
        cost = 0.
        for driver_idx, driver_req_set in enumerate(matching.arrTuple):
            cost += self.problem.COST[driver_idx][driver_req_set]

        return cost

    def checkSolutionExists(self):
        pass


if __name__ == '__main__':
    from generate import DataGenerator

    # nRequests = 100
    # nDrivers = 10

    nRequests = 100
    nDrivers = 10

    seed = 142857
    print('Generating data using seed %d...'%seed)
    generator = DataGenerator(n=nRequests, DD=nDrivers, PRECISION=50, CAP=4, rhoo=1.2, seed=seed)
    drivers, requests = generator.generate()

    policySolver = RandomizedPolicy()
    policySolver.setData(drivers, requests)
    policySolver.preprocess()

    print(policySolver.problem.RTV)

    random.seed(seed)
    baseline_theta = 1./ nRequests
    theta_basic = [random.uniform(baseline_theta*0.3, baseline_theta*1) for j in range(nRequests)]

    # ==== Single theta test === #
    final_objective, final_solution, final_costs, final_matchings = policySolver.solve(theta=theta_basic)


    # === Multiple objectives === #
    objectives = []
    # change_idx = 8
    theta_ratios = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.]
    theta_scaled = deepcopy(theta_basic)
    for theta_ratio in theta_ratios:
        theta_scaled = [theta_basic[i] * theta_ratio for i in range(len(theta_basic))]
        final_objective, final_solution, final_costs, final_matchings = policySolver.solve(theta = theta_scaled)
        objectives.append(final_objective)

    print('objectives:')
    print(objectives)

    print('difference between objectives i and i+1')
    print([objectives[i+1]-objectives[i] for i in range(len(objectives)-1)])


