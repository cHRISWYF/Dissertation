#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug  9 19:24:57 2020

@author: yongfei
"""

import numpy as np
import pandas as pd

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug  5 09:32:07 2020

@author: yongfei
"""
import numpy as np
import pandas as pd
import random


acc_min= -4 # the maximum acceleration m/s2
acc_max= 4# the minimum acceleration m/s2
vee_min=0. # the maximum velocity m/s
vee_max=16.67 # the minimum velocity m/s
headway=2. # the headway of consecutive vehicles s
time_slot=1. # time slot s
   
      
def range_acc(vee):
    '''
    This function is to calculate the real maximun and minimum 
    acceleration/deceleration.
    
    vee: the current velocity of the vehicle
    '''
    
    acc_low_tem = (vee_min - vee) / time_slot
    acc_high_tem = (vee_max - vee) / time_slot
    acc_low = max(acc_min, acc_low_tem)
    acc_high = min(acc_max, acc_high_tem)
    
    return acc_low, acc_high


def entrydistance(s0, vee, acc_next):
    '''
    This function is to update the distance between vehicle and
    the entry of the intersection in the next time slot.
    
    s0: initial distance between the vehicle and the entrance of the intersection
    vee: the current velocity of the vehicle
    acc_next: the allocated acceleration of the vehicle
    s_next: the distance between the vehilce and the entrance of the intersection after adopting the allocated acceleration
    '''
    s_next = s0 - vee * time_slot - 0.5 * acc_next * (time_slot ** 2)
   
    return s_next

def safedistance(vee):
    '''
    This function is to calculate the safe distance
    
    vee: the velocity of the front vehicle 
    '''
    sdt = headway * vee
    
    return sdt



def find_collision_area(k, k_next,indicator, indicator_next):
    '''
    This function is to find out the collision area of vehicles.
    Lengths of collision areas are different due to the asymmetry layout of intersection.
    
    k: the number of the lane of the vehicle
    k_next: the number of the lane of the next vehicle
    indicator: the indicator of the vehicle (turn left, go straight)
    indicator_next: the indicator of the next vehicle
    '''
    global collision_area
    global trajectory 
    global trajectory_next
    global trajectory1
    global trajectory_next1
    
    # trajectory: the length from the entrance of the intersection to the entrance of the collision area for the vehicle
    # trajecory_next: the length from the entrance of the intersection to the entrance of the collision area for the next vehicle
    # trajectory1: the length from the entrance of the intersection to the exit of the collision area for the vehicle
    # trajecory_next1: the length from the entrance of the intersection to the exit of the collision area for the next vehicle

    trajectory, trajectory_next, trajectory1, trajectory_next1= 0, 0, 0, 0
    collision_area = pd.DataFrame({'1': [[4,8],[0,0],[0,0],[7.725,0],[11.325,3.6],[0,0],[0,0],[0,0],[0,0]],
                                      '2': [[4,7],[0,0],[7,4],[5.6625,0],[7.725,3.6],[0,0],[0,0],[5.6625,0],[3.7448,7.725]],
                                      '3': [[4,5],[1,5],[0,0],[3.6,8.7],[5.6625,12.3],[8.7,7.4253],[12.3,11.1701],[0,0],[0,0]],
                                      '4': [[4,6],[0,0],[0,0],[0,8.7],[3.6,12.3],[0,0],[0,0],[0,0],[0,0]],
                                      '5': [[3,8],[0,0],[5,4],[7.725,3.6],[11.325,6.15],[0,0],[0,0],[7.725,7.351],[11.325,11.1701]],
                                      '6': [[3,7],[1,7],[0,0],[5.6625,3.6],[7.725,6.15],[3.8191,5.6625],[7.4253,7.725],[0,0],[0,0]],
                                      '7': [[3,5],[0,0],[3,7],[3.6,6.15],[5.6625,8.7],[0,0],[0,0],[3.6,3.7448],[5.6625,7.351]],
                                      '8': [[3,6],[3,6],[0,0],[0,6.15],[3.6,8.7],[0,6.15],[3.8191,8.7],[0,0],[0,0]],
                                      '9': [[1,8],[1,8],[0,0],[0,6.15],[3.6,8.7],[0,6.15],[3.8191,8.7],[0,0],[0,0]],
                                      '10': [[1,7],[0,0],[3,5],[3.6,6.15],[5.6625,8.7],[0,0],[0,0],[3.6,3.7448],[5.6625,7.351]],
                                      '11': [[1,5],[3,5],[0,0],[5.6625,3.6],[7.725,6.15],[3.8191,3.6],[7.4253,6.15],[0,0],[0,0]],
                                      '12': [[1,6],[0,0],[1,7],[7.725,3.6],[11.325,6.15],[0,0],[0,0],[7.725,7.351],[11.325,11.1701]],
                                      '13': [[2,8],[0,0],[0,0],[0,8.7],[3.6,12.3],[0,0],[0,0],[0,0],[0,0]],
                                      '14': [[2,7],[3,7],[0,0],[3.6,8.7],[5.6625,12.3],[7.4253,8.7],[11.1701,12.3],[0,0],[0,0]],
                                      '15': [[2,5],[0,0],[2,5],[5.6625,0],[7.725,3.6],[0,0],[0,0],[5.6625,0],[7.725,3.7448]],
                                      '16': [[2,6],[0,0],[0,0],[7.725,0],[11.325,3.6],[0,0],[0,0],[0,0],[0,0]]})
    # [straight,straight],[left,straight],[straight,left],[trajectory, trajectory_next],[trajectory_after,trajectory_next_after][trajectory, trajectory_next],[trajectory_after,trajectory_next_after][trajectory, trajectory_next],[trajectory_after,trajectory_next_after]
    if indicator == 'straight' and indicator_next == 'straight':
        for i in range(16):
            if collision_area.iloc[0][i] == [k, k_next] or [k_next, k]:
                area_num = i
        
        if area_num == None:
            print('There is no collision area')
            
        elif k <= 4 and k_next > 4:
            trajectory, trajectory_next = collision_area.iloc[3][area_num]
            trajectory1,trajectory_next1 = collision_area.iloc[4][area_num]
        elif k_next <= 4 and k > 4:
            trajectory_next, trajectory = collision_area.iloc[3][area_num]
            trajectory_next1,trajectory1 = collision_area.iloc[4][area_num]
            
    elif indicator == 'left' and indicator_next == 'straight':
        for i in range(16):
            if collision_area.iloc[1][i] == [k, k_next] or [k_next,k]:
                area_num = i
        
        if area_num == None:
            print('There is no collision area')

            
        elif k <= 4 and k_next > 4:
            trajectory, trajectory_next = collision_area.iloc[5][area_num]
            trajectory1,trajectory_next1 = collision_area.iloc[6][area_num]
        elif k_next <= 4 and k > 4:
            trajectory_next, trajectory = collision_area.iloc[5][area_num]
            trajectory_next1,trajectory1 = collision_area.iloc[6][area_num]
        elif [k, k_next] == [1,3] or [3,1]:
            trajectory, trajectory_next = [3.8191,5.6625]
            trajectory1, trajectory_next1 = [7.4253,7.725]
        elif [k, k_next] == [1,4] or [3,2]:
            trajectory, trajectory_next = [7.4253, 3.6]
            trajectory1, trajectory_next1 = [11.1701,5.6625]
        elif [k, k_next] == [7,5] or [5,7]:
            trajectory, trajectory_next = [3.7448, 6.15]
            trajectory1, trajectory_next1 = [5.5479,7.351]
        elif [k, k_next] == [7,6] or [5,8]:
            trajectory, trajectory_next = [5.5479, 3.6]
            trajectory1, trajectory_next1 = [11.1701,6.15]
            
    elif indicator == 'straight' and indicator_next == 'left':
        for i in range(16):
            if collision_area.iloc[2][i] == [k, k_next] or [k_next,k]:
                area_num = i
        
        if area_num == None:
            print('There is no collision area')
            
        elif k <= 4 and k_next > 4:
            trajectory, trajectory_next = collision_area.iloc[7][area_num]
            trajectory1,trajectory_next1 = collision_area.iloc[8][area_num]
        elif k_next <= 4 and k > 4:
            trajectory_next, trajectory = collision_area.iloc[7][area_num]
            trajectory_next1,trajectory1 = collision_area.iloc[8][area_num]
        elif [k, k_next] == [3,1] or [1,3]:
            trajectory, trajectory_next = [5.6625,3.8191]
            trajectory1, trajectory_next1 = [7.725,7.4253]
        elif [k, k_next] == [4,1] or [2,3]:
            trajectory, trajectory_next = [3.6,7.4253]
            trajectory1, trajectory_next1 = [5.6625,11.1701]
        elif [k, k_next] == [7,5] or [5,7]:
            trajectory, trajectory_next = [6.15,3.7448]
            trajectory1, trajectory_next1 = [7.351,5.5479]
        elif [k, k_next] == [6,7] or [8,5]:
            trajectory, trajectory_next = [3.6,5.5479]
            trajectory1, trajectory_next1 = [6.15,11.1701]
            
    return trajectory, trajectory_next, trajectory1, trajectory_next1

#%%
from scipy.optimize import minimize

def fun(args):
    #a,b,c,d = args
    v = lambda acc: (np.abs(acc[0]) + np.abs(acc[1]) + np.abs(acc[2]) + np.abs(acc[3]))

    return v

# if there is no vehicle in the same lane
def con1(args):
    acc_high1, acc_low1, acc_high2, acc_low2, acc_high3, acc_low3, acc_high4, acc_low4, \
    trajectory_1_12,trajectory_1_13, trajectory_1_14,\
    trajectory_2_23,trajectory_2_24, trajectory_2_21,\
    trajectory_3_34,trajectory_3_31,trajectory_3_32,\
    trajectory_4_41, trajectory_4_42, trajectory_4_43,\
    vee_1, vee_2, vee_3, vee_4, \
    s0_1, s0_2, s0_3, s0_4= args
    '''
    
    acc_high1, acc_low1: the bound of acceleration of the first vehicle 
    trajectory_1_12: the length of trajectory of the first vehicles in collision area in which the first and the second vehicles may collide
    vee_1: the current velocity of the first vehicle
    s0_1: the initial distance of the first vehicls
    
    '''
    cons = (
            {'type':'ineq','fun': lambda acc: acc[0] - acc_low1},\
            {'type':'ineq','fun': lambda acc: -acc[0] + acc_high1},\
            {'type':'ineq','fun': lambda acc: acc[1] - acc_low2},\
            {'type':'ineq','fun': lambda acc: -acc[1] + acc_high2},\
            {'type':'ineq','fun': lambda acc: acc[2] - acc_low3},\
            {'type':'ineq','fun': lambda acc: -acc[2] + acc_high3},\
            {'type':'ineq','fun': lambda acc: acc[3] - acc_low4},\
            {'type':'ineq','fun': lambda acc: -acc[3] + acc_high4},\
            {'type':'ineq','fun': lambda acc: (s0_2-vee_2-(1/2)*acc[1]+trajectory_2_21)/(vee_2+acc[1])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_12)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_3-vee_3-(1/2)*acc[2]+trajectory_3_31)/(vee_3+acc[2])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_13)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_41)/(vee_4+acc[3])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_14)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_3-vee_3-(1/2)*acc[2]+trajectory_3_32)/(vee_3+acc[2])-(s0_2-vee_2-(1/2)*acc[1]+trajectory_2_23)/(vee_2+acc[1])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_42)/(vee_4+acc[3])-(s0_2-vee_2-(1/2)*acc[1]+trajectory_2_24)/(vee_2+acc[1])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_43)/(vee_4+acc[3])-(s0_3-vee_3-(1/2)*acc[2]+trajectory_3_34)/(vee_3+acc[2])}
            )
    
    return cons

# if there is two vehicles in the same lane
def con2(args):
    '''
    a, b: the number of consecutive vehicles, b>a, a-th is the first vehicle and b-th is the latter vehicle in the same lane
    ss0_1, svee_1: the initial distance and velocity of vehicle a
    ss0_2, svee_2: the initial distance and velocity of vehicle b
    '''
    
    acc_high1, acc_low1, acc_high2, acc_low2, acc_low3, acc_high3, acc_high4, acc_low4, \
    trajectory_1_12,trajectory_1_13, trajectory_1_14,\
    trajectory_2_23,trajectory_2_24, trajectory_2_21,\
    trajectory_3_34,trajectory_3_31,trajectory_3_32,\
    trajectory_4_41, trajectory_4_42, trajectory_4_43,\
    vee_1, vee_2, vee_3, vee_4, \
    s0_1, s0_2, s0_3, s0_4,\
    ss0_1, ss0_2, svee_1, svee_2,\
    a, b= args

    cons = (
            {'type':'ineq','fun': lambda acc: acc[0] - acc_low1},\
            {'type':'ineq','fun': lambda acc: -acc[0] + acc_high1},\
            {'type':'ineq','fun': lambda acc: acc[1] - acc_low2},\
            {'type':'ineq','fun': lambda acc: -acc[1] + acc_high2},\
            {'type':'ineq','fun': lambda acc: acc[2] - acc_low3},\
            {'type':'ineq','fun': lambda acc: -acc[2] + acc_high3},\
            {'type':'ineq','fun': lambda acc: acc[3] - acc_low4},\
            {'type':'ineq','fun': lambda acc: -acc[3] + acc_high4},\
            {'type':'ineq','fun': lambda acc: ss0_2-svee_2-(1/2)*acc[b] - ss0_1+svee_1+(1/2)*acc[a] - svee_1 * 0.5 },\
            {'type':'ineq','fun': lambda acc: (s0_2-vee_2-(1/2)*acc[1]+trajectory_2_21)/(vee_2+acc[1])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_12)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_3-vee_3-(1/2)*acc[2]+trajectory_3_31)/(vee_3+acc[2])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_13)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_41)/(vee_4+acc[3])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_14)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_3-vee_3-(1/2)*acc[2]+trajectory_3_32)/(vee_3+acc[2])-(s0_2-vee_2-(1/2)*acc[1]+trajectory_2_23)/(vee_2+acc[1])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_42)/(vee_4+acc[3])-(s0_2-vee_2-(1/2)*acc[1]+trajectory_2_24)/(vee_2+acc[1])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_43)/(vee_4+acc[3])-(s0_3-vee_3-(1/2)*acc[2]+trajectory_3_34)/(vee_3+acc[2])}
            )
 
    
    return cons

# if there is three vhechilces in the same lane
def con3(args):
    '''
    c: c is similar with a,b, c>b>a
    '''
    acc_high1, acc_low1, acc_high2, acc_low2, acc_low3, acc_high3, acc_high4, acc_low4, \
    trajectory_1_12,trajectory_1_13, trajectory_1_14,\
    trajectory_2_23,trajectory_2_24, trajectory_2_21,\
    trajectory_3_34,trajectory_3_31,trajectory_3_32,\
    trajectory_4_41, trajectory_4_42, trajectory_4_43,\
    vee_1, vee_2, vee_3, vee_4, \
    s0_1, s0_2, s0_3, s0_4,\
    ss0_1, ss0_2, ss0_3, svee_1, svee_2, svee_3,\
    a, b, c= args
    cons = (
            {'type':'ineq','fun': lambda acc: acc[0] - acc_low1},\
            {'type':'ineq','fun': lambda acc: -acc[0] + acc_high1},\
            {'type':'ineq','fun': lambda acc: acc[1] - acc_low2},\
            {'type':'ineq','fun': lambda acc: -acc[1] + acc_high2},\
            {'type':'ineq','fun': lambda acc: acc[2] - acc_low3},\
            {'type':'ineq','fun': lambda acc: -acc[2] + acc_high3},\
            {'type':'ineq','fun': lambda acc: acc[3] - acc_low4},\
            {'type':'ineq','fun': lambda acc: -acc[3] + acc_high4},\
            {'type':'ineq','fun': lambda acc: ss0_2-svee_2-(1/2)*acc[b] - ss0_1+svee_1+(1/2)*acc[a] - svee_1 * 0.5 },\
            {'type':'ineq','fun': lambda acc: ss0_3-svee_3-(1/2)*acc[c] - ss0_2+svee_2+(1/2)*acc[b] - svee_2 * 0.5 },\
            {'type':'ineq','fun': lambda acc: (s0_2-vee_2-(1/2)*acc[1]+trajectory_2_21)/(vee_2+acc[1])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_12)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_3-vee_3-(1/2)*acc[2]+trajectory_3_31)/(vee_3+acc[2])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_13)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_41)/(vee_4+acc[3])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_14)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_3-vee_3-(1/2)*acc[2]+trajectory_3_32)/(vee_3+acc[2])-(s0_2-vee_2-(1/2)*acc[1]+trajectory_2_23)/(vee_2+acc[1])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_42)/(vee_4+acc[3])-(s0_2-vee_2-(1/2)*acc[1]+trajectory_2_24)/(vee_2+acc[1])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_43)/(vee_4+acc[3])-(s0_3-vee_3-(1/2)*acc[2]+trajectory_3_34)/(vee_3+acc[2])}
            )
    
    return cons

# if two vhechilces are in the same lane and another two vehicle are in the same lane
def con4(args):
    '''
    d:  d>c>b>a
    '''
    acc_high1, acc_low1, acc_high2, acc_low2, acc_low3, acc_high3, acc_high4, acc_low4, \
    trajectory_1_12,trajectory_1_13, trajectory_1_14,\
    trajectory_2_23,trajectory_2_24, trajectory_2_21,\
    trajectory_3_34,trajectory_3_31,trajectory_3_32,\
    trajectory_4_41, trajectory_4_42, trajectory_4_43,\
    vee_1, vee_2, vee_3, vee_4, \
    s0_1, s0_2, s0_3, s0_4,\
    ss0_1, ss0_2, ss0_3, ss0_4, svee_1, svee_2, svee_3, svee_4,\
    a, b, c, d= args
    cons = (
            {'type':'ineq','fun': lambda acc: acc[0] - acc_low1},\
            {'type':'ineq','fun': lambda acc: -acc[0] + acc_high1},\
            {'type':'ineq','fun': lambda acc: acc[1] - acc_low2},\
            {'type':'ineq','fun': lambda acc: -acc[1] + acc_high2},\
            {'type':'ineq','fun': lambda acc: acc[2] - acc_low3},\
            {'type':'ineq','fun': lambda acc: -acc[2] + acc_high3},\
            {'type':'ineq','fun': lambda acc: acc[3] - acc_low4},\
            {'type':'ineq','fun': lambda acc: -acc[3] + acc_high4},\
            {'type':'ineq','fun': lambda acc: ss0_2-svee_2-(1/2)*acc[b] - ss0_1+svee_1+(1/2)*acc[a] - svee_1 * 0.5 },\
            {'type':'ineq','fun': lambda acc: ss0_4-svee_4-(1/2)*acc[d] - ss0_3+svee_3+(1/2)*acc[c] - svee_3 * 0.5 },\
            {'type':'ineq','fun': lambda acc: (s0_2-vee_2-(1/2)*acc[1]+trajectory_2_21)/(vee_2+acc[1])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_12)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_3-vee_3-(1/2)*acc[2]+trajectory_3_31)/(vee_3+acc[2])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_13)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_41)/(vee_4+acc[3])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_14)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_3-vee_3-(1/2)*acc[2]+trajectory_3_32)/(vee_3+acc[2])-(s0_2-vee_2-(1/2)*acc[1]+trajectory_2_23)/(vee_2+acc[1])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_42)/(vee_4+acc[3])-(s0_2-vee_2-(1/2)*acc[1]+trajectory_2_24)/(vee_2+acc[1])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_43)/(vee_4+acc[3])-(s0_3-vee_3-(1/2)*acc[2]+trajectory_3_34)/(vee_3+acc[2])}
            )
    
    return cons


# if all vhechilces in the same lane
def con5(args):
   
    acc_high1, acc_low1, acc_high2, acc_low2, acc_low3, acc_high3, acc_high4, acc_low4, \
    trajectory_1_12,trajectory_1_13, trajectory_1_14,\
    trajectory_2_23,trajectory_2_24, trajectory_2_21,\
    trajectory_3_34,trajectory_3_31,trajectory_3_32,\
    trajectory_4_41, trajectory_4_42, trajectory_4_43,\
    vee_1, vee_2, vee_3, vee_4, \
    s0_1, s0_2, s0_3, s0_4 = args
    cons = (
            {'type':'ineq','fun': lambda acc: acc[0] - acc_low1},\
            {'type':'ineq','fun': lambda acc: -acc[0] + acc_high1},\
            {'type':'ineq','fun': lambda acc: acc[1] - acc_low2},\
            {'type':'ineq','fun': lambda acc: -acc[1] + acc_high2},\
            {'type':'ineq','fun': lambda acc: acc[2] - acc_low3},\
            {'type':'ineq','fun': lambda acc: -acc[2] + acc_high3},\
            {'type':'ineq','fun': lambda acc: acc[3] - acc_low4},\
            {'type':'ineq','fun': lambda acc: -acc[3] + acc_high4},\
            {'type':'ineq','fun': lambda acc: s0_2-vee_2-(1/2)*acc[1] - s0_1+vee_1+(1/2)*acc[0] - vee_1 * 0.5 },\
            {'type':'ineq','fun': lambda acc: s0_3-vee_3-(1/2)*acc[2] - s0_2+vee_2+(1/2)*acc[1] - vee_2 * 0.5 },\
            {'type':'ineq','fun': lambda acc: s0_4-vee_4-(1/2)*acc[3] - s0_3+vee_3+(1/2)*acc[2] - vee_3 * 0.5 },\
            {'type':'ineq','fun': lambda acc: (s0_2-vee_2-(1/2)*acc[1]+trajectory_2_21)/(vee_2+acc[1])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_12)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_3-vee_3-(1/2)*acc[2]+trajectory_3_31)/(vee_3+acc[2])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_13)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_41)/(vee_4+acc[3])-(s0_1-vee_1-(1/2)*acc[0]+trajectory_1_14)/(vee_1+acc[0])},\
            {'type':'ineq','fun': lambda acc: (s0_3-vee_3-(1/2)*acc[2]+trajectory_3_32)/(vee_3+acc[2])-(s0_2-vee_2-(1/2)*acc[1]+trajectory_2_23)/(vee_2+acc[1])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_42)/(vee_4+acc[3])-(s0_2-vee_2-(1/2)*acc[1]+trajectory_2_24)/(vee_2+acc[1])},\
            {'type':'ineq','fun': lambda acc: (s0_4-vee_4-(1/2)*acc[3]+trajectory_4_43)/(vee_4+acc[3])-(s0_3-vee_3-(1/2)*acc[2]+trajectory_3_34)/(vee_3+acc[2])}
            )
    
    return cons
    
    
#%%    
'''
generate data randomly
'''  
vehicle_info = pd.DataFrame(columns=('number','lane','indicator','acceleration','velocity','initial distance'))
time = []
time_tem = 0
indicator = []
k = []

random.seed = (1)
for i in range(1,601):
    k_tmp = random.randint(1,8)
    if k_tmp % 2 == 1:
        indicator_tmp = random.choice(['left','straight'])
    else:
        indicator_tmp = 'straight'
       
    a = np.random.uniform(acc_min, acc_max)
    v = np.random.uniform(vee_min, vee_max)
    s0 = np.random.uniform(50,100)
    
    vehicle_info.loc[i] = [i, k_tmp, indicator_tmp, a, v, s0]
    #vehicle_info.iloc[i][6] = time_tem

#%%
'''
in general, the sum of the acceleration and the velocity is postive to avoid negative velocity in the next period 
'''
error = []
for i in range(len(vehicle_info)):
    if vehicle_info.iloc[i]['acceleration'] + vehicle_info.iloc[i]['velocity'] <= 0:
        tmp = np.random.uniform(np.abs(vehicle_info.iloc[i]['acceleration']),vee_max)
        vehicle_info.loc[i+1,'velocity'] = tmp
        error.append(i)

#%%
acc_solution = []
fuel_consumption = 0
failure = []
for i in range(0,150):
    vehicle =vehicle_info.iloc[i*4:(i*4+4),:]
    vehicle = vehicle.sort_values('initial distance')
    #vehicle.reset_index(drop=True)
    vehicle.index = range(1,5)
    
    acc_low = []
    acc_high = []
    for j in range(1,5):
        acc_low_tem = range_acc(vehicle.loc[j]['velocity'])[0]
        acc_high_tem = range_acc(vehicle.loc[j]['velocity'])[1]
        acc_low.append(acc_low_tem)
        acc_high.append(acc_high_tem)
        
    trajectory_1_12, trajectory_2, trajectory1_1, trajectory_2_21 = find_collision_area(vehicle.loc[1]['lane'],vehicle.loc[2]['lane'],vehicle.loc[1]['indicator'],vehicle.loc[2]['indicator'])
    trajectory_1_13, trajectory_3, trajectory1_1, trajectory_3_31 = find_collision_area(vehicle.loc[1]['lane'],vehicle.loc[3]['lane'],vehicle.loc[1]['indicator'],vehicle.loc[3]['indicator'])
    trajectory_1_14, trajectory_4, trajectory1_1, trajectory_4_41 = find_collision_area(vehicle.loc[1]['lane'],vehicle.loc[4]['lane'],vehicle.loc[1]['indicator'],vehicle.loc[4]['indicator'])
    trajectory_2_23, trajectory_3, trajectory1_2, trajectory_3_32 = find_collision_area(vehicle.loc[2]['lane'],vehicle.loc[3]['lane'],vehicle.loc[2]['indicator'],vehicle.loc[3]['indicator'])
    trajectory_2_24, trajectory_4, trajectory1_2, trajectory_4_42 = find_collision_area(vehicle.loc[2]['lane'],vehicle.loc[4]['lane'],vehicle.loc[2]['indicator'],vehicle.loc[4]['indicator'])
    trajectory_3_34, trajectory_4, trajectory1_3, trajectory_4_43 = find_collision_area(vehicle.loc[3]['lane'],vehicle.loc[4]['lane'],vehicle.loc[3]['indicator'],vehicle.loc[4]['indicator'])

    vee_1 = vehicle.loc[1]['velocity']
    vee_2 = vehicle.loc[2]['velocity']
    vee_3 = vehicle.loc[3]['velocity']
    vee_4 = vehicle.loc[4]['velocity']
    s0_1 = vehicle.loc[1]['initial distance']
    s0_2 = vehicle.loc[2]['initial distance']
    s0_3 = vehicle.loc[3]['initial distance']
    s0_4 = vehicle.loc[4]['initial distance']
    
    condition = len(vehicle['lane'].unique())
    
    if condition == 4:
        if __name__ == '__main__':
            args1 = (acc_high[0],acc_low[0],acc_high[1],acc_low[1],acc_high[2],acc_low[2],acc_high[3],acc_low[3],\
                         trajectory_1_12,trajectory_1_13, trajectory_1_14,\
                         trajectory_2_23,trajectory_2_24, trajectory_2_21,\
                         trajectory_3_34,trajectory_3_31,trajectory_3_32,\
                         trajectory_4_41, trajectory_4_42, trajectory_4_43,\
                         vee_1 ,vee_2 ,vee_3,vee_4,s0_1,s0_2 ,s0_3, s0_4)
                
            cons = con1(args1)
            acc = [vehicle.loc[1]['acceleration'],vehicle.loc[2]['acceleration'],vehicle.loc[3]['acceleration'],vehicle.loc[4]['acceleration']]
            
            res = minimize(fun(args1), acc, method='SLSQP',constraints=cons)
            
            print('the cycle:', i)
            print(res.fun)
            print(res.success)
            if res.success == 'False':
                failure.append(i)
            print(res.x)
            for sol in res.x:
                acc_solution.append(sol)
                
    elif condition == 3:
        
        a1 = pd.DataFrame(vehicle['lane'].value_counts())
        b1 = a1['lane'].idxmax()
        order = []
        
        for j in range(4):
            if vehicle.iloc[j]['lane'] == b1:
                order.append(j)
                
        if __name__ == '__main__':
           
            ss0_1 = vehicle.iloc[order[0]]['initial distance']
            ss0_2 = vehicle.iloc[order[1]]['initial distance']
            svee_1 = vehicle.iloc[order[0]]['velocity']
            svee_2 = vehicle.iloc[order[1]]['velocity']
            a = order[0]
            b = order[1]
            args1 = (acc_high[0],acc_low[0],acc_high[1],acc_low[1],acc_high[2],acc_low[2],acc_high[3],acc_low[3],\
                         trajectory_1_12,trajectory_1_13, trajectory_1_14,\
                         trajectory_2_23,trajectory_2_24, trajectory_2_21,\
                         trajectory_3_34,trajectory_3_31,trajectory_3_32,\
                         trajectory_4_41, trajectory_4_42, trajectory_4_43,\
                         vee_1 ,vee_2 ,vee_3,vee_4,s0_1,s0_2 ,s0_3, s0_4,
                         ss0_1, ss0_2, svee_1, svee_2,\
                         a, b
                         )
                
            cons = con2(args1)
            acc = [vehicle.loc[1]['acceleration'],vehicle.loc[2]['acceleration'],vehicle.loc[3]['acceleration'],vehicle.loc[4]['acceleration']]
            
            res = minimize(fun(args1), acc, method='SLSQP',constraints=cons)
            
            print('the cycle:', i)
            print(res.fun)
            print(res.success)
            if res.success == 'False':
                failure.append(i)
            
            print(res.x)
            for sol in res.x:
                acc_solution.append(sol)
        
    elif condition == 2:
        
        a2 = pd.DataFrame(vehicle['lane'].value_counts())
        
        if a2.iloc[0]['lane'] != a2.iloc[1]['lane']:
            
            b2 = a2['lane'].idxmax()
            order = []
            
            for j in range(4):
                if vehicle.iloc[j]['lane'] == b2:
                    order.append(j)
                    
            if __name__ == '__main__':
                
              
                ss0_1 = vehicle.iloc[order[0]]['initial distance']
                ss0_2 = vehicle.iloc[order[1]]['initial distance']
                ss0_3 = vehicle.iloc[order[2]]['initial distance']
                svee_1 = vehicle.iloc[order[0]]['velocity']
                svee_2 = vehicle.iloc[order[1]]['velocity']
                svee_3 = vehicle.iloc[order[2]]['velocity']

                a = order[0]
                b = order[1]
                c = order[2]
                args1 = (acc_high[0],acc_low[0],acc_high[1],acc_low[1],acc_high[2],acc_low[2],acc_high[3],acc_low[3],\
                             trajectory_1_12,trajectory_1_13, trajectory_1_14,\
                             trajectory_2_23,trajectory_2_24, trajectory_2_21,\
                             trajectory_3_34,trajectory_3_31,trajectory_3_32,\
                             trajectory_4_41, trajectory_4_42, trajectory_4_43,\
                             vee_1 ,vee_2 ,vee_3,vee_4,s0_1,s0_2 ,s0_3, s0_4,
                             ss0_1, ss0_2, ss0_3, svee_1, svee_2, svee_3,\
                             a, b, c
                             )
                    
                cons = con3(args1)
                acc = [vehicle.loc[1]['acceleration'],vehicle.loc[2]['acceleration'],vehicle.loc[3]['acceleration'],vehicle.loc[4]['acceleration']]
                
                res = minimize(fun(args1), acc, method='SLSQP',constraints=cons)
                
                print('the cycle:', i)
                print(res.fun)
                print(res.success)
                if res.success == 'False':
                    failure.append(i)
                print(res.x)
                for sol in res.x:
                    acc_solution.append(sol)
            
        else:
           c2 = a2.index.values
           order1 = []
           order2 = []
           for j in range(4):
               if vehicle.iloc[j]['lane'] == c2[0]:
                   order1.append(j)
               elif vehicle.iloc[j]['lane'] == c2[1]:
                   order2.append(j)
           if __name__ == '__main__':
               ss0_1 = vehicle.iloc[order1[0]]['initial distance']
               ss0_2 = vehicle.iloc[order1[1]]['initial distance']
               ss0_3 = vehicle.iloc[order2[0]]['initial distance']
               ss0_4 = vehicle.iloc[order2[1]]['initial distance']
               svee_1 = vehicle.iloc[order1[0]]['velocity']
               svee_2 = vehicle.iloc[order1[1]]['velocity']
               svee_3 = vehicle.iloc[order2[0]]['velocity']
               svee_4 = vehicle.iloc[order2[1]]['velocity']
               
               a = order1[0]
               b = order1[1]
               c = order2[0]
               d = order2[1]


               args1 = (acc_high[0],acc_low[0],acc_high[1],acc_low[1],acc_high[2],acc_low[2],acc_high[3],acc_low[3],\
                             trajectory_1_12,trajectory_1_13, trajectory_1_14,\
                             trajectory_2_23,trajectory_2_24, trajectory_2_21,\
                             trajectory_3_34,trajectory_3_31,trajectory_3_32,\
                             trajectory_4_41, trajectory_4_42, trajectory_4_43,\
                             vee_1 ,vee_2 ,vee_3,vee_4,s0_1,s0_2 ,s0_3, s0_4,\
                             ss0_1, ss0_2, ss0_3, ss0_4,svee_1, svee_2, svee_3,svee_4,\
                             a, b, c, d
                             )
                    
               cons = con4(args1)
               acc = [vehicle.loc[1]['acceleration'],vehicle.loc[2]['acceleration'],vehicle.loc[3]['acceleration'],vehicle.loc[4]['acceleration']]
                
               res = minimize(fun(args1), acc, method='SLSQP',constraints=cons)
                
               print('the cycle:', i)
               print(res.fun)
               print(res.success)
               if res.success == 'False':
                   failure.append(i)
               print(res.x)
               for sol in res.x:
                   acc_solution.append(sol)
                   
    elif condition == 1:
        
         if __name__ == '__main__':
            args1 = (acc_high[0],acc_low[0],acc_high[1],acc_low[1],acc_high[2],acc_low[2],acc_high[3],acc_low[3],\
                         trajectory_1_12,trajectory_1_13, trajectory_1_14,\
                         trajectory_2_23,trajectory_2_24, trajectory_2_21,\
                         trajectory_3_34,trajectory_3_31,trajectory_3_32,\
                         trajectory_4_41, trajectory_4_42, trajectory_4_43,\
                         vee_1 ,vee_2 ,vee_3,vee_4,s0_1,s0_2 ,s0_3, s0_4)
                
            cons = con5(args1)
            acc = [vehicle.loc[1]['acceleration'],vehicle.loc[2]['acceleration'],vehicle.loc[3]['acceleration'],vehicle.loc[4]['acceleration']]
            
            res = minimize(fun(args1), acc, method='SLSQP',constraints=cons)
            
            print('the cycle:', i)
            print(res.fun)
            print(res.success)
            if res.success == 0:
                failure.append(i)
            print(res.x)
            for sol in res.x:
                acc_solution.append(sol)
    
'''
TNC : truncated newton algorithm
COBYLA: constrained optimization by linear approximation algorith
doglog: the dog-leg trust-region algorithm
trust-ncg: the Newton conjugate gradient trust-region algorithm.

'''

#%%
# the plot of the ditribution of initial acceleration and new acceleration
import matplotlib.pyplot as plt
import itertools

        
x1 = np.arange(600)
y1 = acc_solution
acc_initial = []
initial_distance = []
initial_velocity = []
for i in range(0,150):
    vehicle =vehicle_info.iloc[i*4:(i*4+4),:]
    vehicle = vehicle.sort_values('initial distance')
    #vehicle.reset_index(drop=True)
    vehicle.index = range(1,5)
    acc_initial.append(vehicle['acceleration'])
    initial_distance.append(vehicle['initial distance'])
    initial_velocity.append(vehicle['velocity'])
    
#y2 = vehicle_info['acceleration']
acc_initial = list(itertools.chain.from_iterable(acc_initial))
initial_distance = list(itertools.chain.from_iterable(initial_distance))
initial_velocity = list(itertools.chain.from_iterable(initial_velocity))
y2 = acc_initial

colors1 = 'blue'
colors2 = 'red'
area = np.pi * 3**3

plt.xlabel('Vehicle')
plt.ylabel('Acceleration (m/s2)')
plt.scatter(x1, y1, s=area, alpha=0.4, marker='.',label='New Acceleration')
plt.scatter(x1, y2, s=area, c=colors2, alpha=0.4, marker='.', label='Initial Acceleration')
plt.ylim(-5,5)
#plt.ylim(-5,5)
#plt.plot([0,0],[600,0],linewidth = '1')
plt.legend(loc='upper right')
plt.show()

#%%
# the plot of the ditribution of the intial and the new acceleration of the first 24 vehicles

x2 = np.arange(24)
y3 = acc_solution[:24]
y4 = acc_initial[:24]

x_align = np.arange(0,25, 4)
y_align = [5] * 7

plt.scatter(x2, y3, s=area, alpha=0.4, marker='.',label='New Acceleration')
plt.scatter(x2, y4, s=area, c=colors2, alpha=0.4, marker='.', label='Initial Acceleration')

for i in range(len(x2)):
    x_tmp = [x2[i],x2[i]]
    y_tmp = [y3[i],y4[i]]
    plt.plot(x_tmp,y_tmp)
   
for i in range(len(x_align)):
    x_tmp = [(x_align[i]-0.5),(x_align[i]-0.5)]
    y_tmp = [-5, y_align[i]]
    plt.plot(x_tmp,y_tmp,'k--')

plt.xlabel('Vehicle')
plt.ylabel('Acceleration (m/s2)')
#%%

total_consumtion_before = np.sum(np.abs(vehicle_info['acceleration']))
total_consumption_after = np.sum(np.abs(acc_solution))
#%%
cycle = 0
for i in range(len(acc_solution)):
    s_traj = []
    
    s_traj0 = initial_distance[i]
    s_traj1 = s_traj0 - initial_velocity[i] * 1 - 0.5 * acc_solution[i]
    v_new = initial_velocity[i] + acc_solution[i]
    s_traj.append(s_traj0)
    s_traj.append(s_traj1)
    s_traj_tmp = s_traj1
    for j in range(10):
        s_traj_tmp -= v_new * 1
        s_traj.append(s_traj_tmp)

    if i % 4 == 0:
        cycle +=1
        x= np.arange(12*cycle,12*(cycle+1))
        
    plt.plot(x,s_traj)
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
#%%
cyc = 0
for i in range(88,92):
    s_traj = []
    
    s_traj0 = initial_distance[i]
    s_traj1 = s_traj0 - initial_velocity[i] * 1 - 0.5 * acc_solution[i]
    v_new = initial_velocity[i] + acc_solution[i]
    s_traj.append(s_traj0)
    s_traj.append(s_traj1)
    s_traj_tmp = s_traj1
    for j in range(15):
        s_traj_tmp -= v_new * 1
        s_traj.append(s_traj_tmp)
  
    if i % 4 == 0:
        cyc +=1
        x= np.arange(17*cyc,17*(cyc+1))
        
    plt.plot(x,s_traj)
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
#%%

total_consumption_iniital = np.sum(np.abs(vehicle_info['acceleration']))
total_consumption_after = np.sum(np.abs(acc_solution))

    
  






