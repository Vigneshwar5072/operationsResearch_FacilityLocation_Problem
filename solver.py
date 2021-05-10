  #!/usr/bin/python3
# -*- coding: utf-8 -*-

import math
import numpy as np
from psutil import cpu_count
from collections import namedtuple
from gurobipy import *

Point = namedtuple("Point", ['x', 'y'])
Facility = namedtuple("Facility", ['index', 'setup_cost', 'capacity', 'location'])
Customer = namedtuple("Customer", ['index', 'demand', 'location'])


def dist(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def solve_it(input_data):
    # Modify this code to run your optimization algorithm

    # parse the input
    lines = input_data.split('\n')

    parts = lines[0].split()
    facility_count = int(parts[0])
    customer_count = int(parts[1])
    
    facilities = []
    for i in range(1, facility_count+1):
        parts = lines[i].split()
        facilities.append(Facility(i-1, float(parts[0]), int(parts[1]), Point(float(parts[2]), float(parts[3])) ))

    customers = []
    for i in range(facility_count+1, facility_count+1+customer_count):
        parts = lines[i].split()
        customers.append(Customer(i-1-facility_count, int(parts[0]), Point(float(parts[1]), float(parts[2]))))

    # trivial solution
    # pack the facilities one by one until all the customers are served
    # ==========
    
    
    obj, opt, solution = initialSearch(facilities, customers)
    #obj, opt, solution = trivial(facilities, customers)

    obj, opt, solution = mip(facilities, customers,
                             verbose=False,
                             time_limit=1800)

    # prepare the solution in the specified output format
    output_data = '%.2f' % obj + ' ' + str(opt) + '\n'
    output_data += ' '.join(map(str, solution))

    return output_data



def mip(facilities, customers, verbose=False, num_threads=None, time_limit=None):
    f_count = len(facilities)
    c_count = len(customers)
# =============================================================================
#     print(customers)
#     print(facilities)
#     print(customers[1].location)
#     print(facilities[1].location)
# =============================================================================

    m = Model("facility_location")
    m.setParam('OutputFlag', verbose)
    if num_threads:
        m.setParam("Threads", num_threads)
    else:
        m.setParam("Threads", cpu_count())

    if time_limit:
        m.setParam("TimeLimit", time_limit)

    x = m.addVars(f_count, vtype=GRB.BINARY, name="x")
    y = m.addVars(c_count, f_count, vtype=GRB.BINARY, name="y")

    m.setObjective(LinExpr((facilities[j].setup_cost, x[j])
                           for j in range(f_count)) +
                   LinExpr((dist(customers[i].location, facilities[j].location), y[(i, j)])
                           for i in range(c_count)
                           for j in range(f_count)),
                   GRB.MINIMIZE)

    m.addConstrs((y.sum(i, "*") == 1
                  for i in range(c_count)),
                 name="assign_constr")

    m.addConstrs((x[j] >= y.sum(i, j)
                  for i in range(c_count)
                  for j in range(f_count)),
                 name="xy_corr_constr")

    m.addConstrs((LinExpr((customers[i].demand, y[(i, j)])
                          for i in range(c_count)) <= facilities[j].capacity
                  for j in range(f_count)),
                 name="cap_constr")

    m.update()
    m.optimize()

    total_cost = m.getObjective().getValue()
    isol = [[int(m.getVarByName("y[{},{}]".format(i, j)).x)
             for j in range(f_count)]
            for i in range(c_count)]
    soln = [j for i in range(c_count) for j in range(f_count) if isol[i][j] == 1]

    if m.status == 2:
        opt = 1
    else:
        opt = 0

    return total_cost, opt, soln


def initialSearch(facilities, customers):
    alpha = 0.05
    #iS_distance_matrix = []

    f_count = len(facilities)
    c_count = len(customers)
    iS_distance_matrix = np.empty(shape = (c_count, f_count), dtype = object)
    for i in range(c_count):
        for j in range(f_count):
            iS_distance_matrix[i][j] =  dist(customers[i].location, facilities[j].location)
    
    print(iS_distance_matrix)        
    
    iS_initial_assignment(customers, facilities, iS_distance_matrix, f_count, c_count)



#initializing assignment through greedy algorithm
def iS_initial_assignment(customers, facilities, iS_distance_matrix, f_count, c_count):
    for i in range(c_count):
        #customer_index = 0
        customer = i
        min_distance = float('inf')
        min_facility = -1
        for j in range(f_count):
            #facility_index = 0
            facility = j
            print(customer, facility)
            print(customers, facilities)
            if(min_distance > iS_distance_matrix[customer][facility] 
               and customers[customer].demand <= facilities[facility].capacity):
                min_distance = iS_distance_matrix[customer][facility]
                min_facility = facility
                
                #facility_index += 1
        facilities[min_facility].customers.insert(customer)
        facilities[min_facility].capacity -= customers[customer].demand
        
        customers[customer].facility = min_facility
        #customer_index +=1
                

def trivial(facilities, customers):
    solution = [-1] * len(customers)
    capacity_remaining = [f.capacity for f in facilities]
    facility_index = 0
    for customer in customers:
        if capacity_remaining[facility_index] >= customer.demand:
            solution[customer.index] = facility_index
            capacity_remaining[facility_index] -= customer.demand
        else:
            facility_index += 1
            assert capacity_remaining[facility_index] >= customer.demand
            solution[customer.index] = facility_index
            capacity_remaining[facility_index] -= customer.demand

    used = [0]*len(facilities)
    for facility_index in solution:
        used[facility_index] = 1

    obj = sum([f.setup_cost*used[f.index] for f in facilities])
    for customer in customers:
        obj += dist(customer.location, facilities[solution[customer.index]].location)

    return obj, 0, solution


# =============================================================================
# if __name__ == '__main__':
#     import sys
#     if len(sys.argv) > 1:
#         file_location = sys.argv[1].strip()
#         with open(file_location, 'r') as input_data_file:
#             input_data = input_data_file.read()
#         print(solve_it(input_data))
#     else:
#         print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/fl_16_2)')
# =============================================================================

if __name__ == '__main__':
    import sys
    #if len(sys.argv) > 1:
    #   file_location = sys.argv[1].strip()
    file_location = 'C:/Users/VigneshwarPesaru/Downloads/facility_location/facility/data/fl_16_1'
    with open(file_location, 'r') as input_data_file:
        #print(input_data_file)
        input_data = input_data_file.read()
        print(solve_it(input_data))
    #else:
    #    print('This test requires an input file.  Please select one from the data directory. (i.e. python solver.py ./data/gc_4_1)')

