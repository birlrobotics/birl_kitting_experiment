#!/usr/bin/env python
import numpy
import os
import glob
import numpy
import birl_dmp
import ipdb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
from smach_based_introspection_framework.offline_part.model_training import train_dmp_model
import dill
from birl_dmp.dmp_training.util import generalize_via_dmp

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
demonstrations_dir = os.path.join(dir_of_this_script, '..', 'data', 'demonstrations')
dmp_model_dir = os.path.join(dir_of_this_script, '..', 'data', 'dmp_models')
dmp_model_profiles = os.path.join(dir_of_this_script, '..', 'data', 'dmp_model_profiles')

def profile_dmp(list_of_mat, dmp_model, fname):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')  
    for mat in list_of_mat:
        ax.plot(mat[:, 0], mat[:, 1], mat[:, 2], color='black', label="raw")

    for mat in list_of_mat:
        start = mat[0].copy()
        end = mat[-1].copy()
        gen_mat = generalize_via_dmp(start, end, dmp_model)
        ax.plot(gen_mat[:, 0], gen_mat[:, 1], gen_mat[:, 2], color='gold',label = 'gen_with_raw_st_end')

    for i in numpy.arange(0.01, 0.1, 0.01):
        print i
        for mat in list_of_mat:
            start = mat[0].copy()
            end = mat[-1].copy()
            end[2] += i
            gen_mat = generalize_via_dmp(start, end, dmp_model)
            ax.plot(gen_mat[:, 0], gen_mat[:, 1], gen_mat[:, 2], color='blue', label='generation')

    for mat in list_of_mat:
        for i in range(3):
            start = mat[0].copy()
            #start = mat[0].copy()+numpy.random.normal(0,pow(0.1,i),mat.shape[1])
            end = mat[-1].copy()+numpy.random.normal(0,pow(0.1,i),mat.shape[1])
            gen_mat = generalize_via_dmp(start, end, dmp_model)
            ax.plot(gen_mat[:, 0], gen_mat[:, 1], gen_mat[:, 2], color='red',label = 'useless')

    ax.set_xlim3d(0, 2)
    ax.set_ylim3d(-2, 2)
    ax.set_zlim3d(-2, 2)
    ax.set_title(fname)
    plt.legend()
    fig.show()

def filter_static_points(mat):
    last = mat[0]
    new_mat = [last]
    for idx in range(mat.shape[0]):
        if numpy.linalg.norm(mat[idx]-last) < 0.01:
            pass
        else:
            new_mat.append(mat[idx])
            last = mat[idx] 

    return numpy.array(new_mat)


if __name__ == '__main__':
    d = {}
    for f in glob.glob(os.path.join(demonstrations_dir, '*.npy')):
        label = os.path.basename(f)[:-4].split('.')[0]
        if label not in d:
            d[label] = []
        d[label].append(f)
    print d

    for label in d:
        if os.path.exists(os.path.join(dmp_model_dir, label)):
            print "model exist, ganna to skip"
            continue
        list_of_mat = [filter_static_points(numpy.load(f)) for f in d[label]]

        orig_mat = list_of_mat[0]    

        result = train_dmp_model.run(orig_mat)

        if not os.path.isdir(dmp_model_dir):
            os.makedirs(dmp_model_dir)
        dill.dump(
            result['model'],
            open(os.path.join(dmp_model_dir, label), 'w')
        )

        profile_dmp(list_of_mat, result['model'], label)
    
    raw_input("press any key to exit")
