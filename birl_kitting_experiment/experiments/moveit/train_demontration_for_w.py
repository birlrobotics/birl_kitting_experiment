#!/usr/bin/env python 
import numpy as np
import pandas as pd
import os
from birl_baxter_dmp.dmp_train import train

    
def get_parameter_w(limb_name, data_path, distance):
    train_set = pd.read_csv(data_path)  # using pandas read data
    train_len = len(train_set)
    if limb_name == "left":
        postion_x = np.array(train_set['position_left_x'])
        postion_y = np.array(train_set['position_left_y'])
        postion_z = np.array(train_set['position_left_z'])
        orientation_x = np.array(train_set['orientation_left_x'])
        orientation_y = np.array(train_set['orientation_left_y'])
        orientation_z = np.array(train_set['orientation_left_z'])
        orientation_w = np.array(train_set['orientation_left_w'])
    if limb_name == "right":
        postion_x = np.array(train_set['position_right_x'])
        postion_y = np.array(train_set['position_right_y'])
        postion_z = np.array(train_set['position_right_z'])
        orientation_x = np.array(train_set['orientation_right_x'])
        orientation_y = np.array(train_set['orientation_right_y'])
        orientation_z = np.array(train_set['orientation_right_z'])
        orientation_w = np.array(train_set['orientation_right_w'])
        
    new_data = []
    ref = [postion_x[0], postion_y[0] ,postion_z[0], orientation_x[0],orientation_y[0],orientation_z[0],orientation_w[0]] # set referenece data
    new_data.append(ref)
    for idx in range(1,len(train_set)):    
        # caculate distrance bettwen two points
        dis = np.sqrt(np.square(ref[0]-postion_x[idx]) + np.square(ref[1]-postion_y[idx]) + np.square(ref[2]-postion_z[idx]))
        if dis > distance: # set offset, make points more sparse
            new_data.append([postion_x[idx], postion_y[idx] ,postion_z[idx], orientation_x[idx],orientation_y[idx],orientation_z[idx],orientation_w[idx]])
            ref = [postion_x[idx], postion_y[idx] ,postion_z[idx], orientation_x[idx],orientation_y[idx],orientation_z[idx],orientation_w[idx]] # if this point meet requiremrnt. them choose this as reference point
    
    new_x = []
    new_y = []
    new_z = []
    new_orin_x = []
    new_orin_y = []
    new_orin_z = []
    new_orin_w = []
    
    for i in range(len(new_data)):
        new_x.append(new_data[i][0])    
        new_y.append(new_data[i][1])
        new_z.append(new_data[i][2])
        new_orin_x.append(new_data[i][3])
        new_orin_y.append(new_data[i][4])
        new_orin_z.append(new_data[i][5])
        new_orin_w.append(new_data[i][6])
    new_x = np.array(new_x)
    new_y = np.array(new_y)
    new_z = np.array(new_z)
    new_orin_x = np.array(new_orin_x)
    new_orin_y = np.array(new_orin_y)
    new_orin_z = np.array(new_orin_z)
    new_orin_w = np.array(new_orin_w)
        
    start_of_demo = new_data[0]
    end_of_demo = new_data[-1]
    train_set_T = np.array(
        [np.array([new_x, new_y, new_z, new_orin_x, new_orin_y, new_orin_z, new_orin_w]).T])
    dmp_w, base_function = train(train_set_T)
    return dmp_w,start_of_demo,end_of_demo 
    
