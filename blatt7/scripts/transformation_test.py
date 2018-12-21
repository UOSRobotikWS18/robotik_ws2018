#!/usr/bin/env python

import numpy as np

def find_transformation(past_pcl, curr_pcl, correspondences):

    N = correspondences.shape[1]

    A = np.zeros((N*2, 6))
    b = np.zeros((N*2, 1))

    # fill A
    X = np.ones( (N,3) )
    X[:,:2] = curr_pcl[correspondences[1,:]]

    A[:N,:3] = X
    A[N:,3:] = X

    # fill b
    b[:N,0] = past_pcl[correspondences[0,:],0]
    b[N:,0] = past_pcl[correspondences[0,:],1]

    # linear least squares
    # x = (A^t*A)^(-1)*A^t*b 

    B = np.matmul(A.transpose(), A)
    c = np.matmul(A.transpose(), b)

    x = np.matmul(np.linalg.inv(B), c)

    trans = np.array([x[2], x[5]])

    theta = np.arcsin(np.round(-x[1,0],4))

    return trans, theta

def distance(p, q):
    """
    Euclidean distance for point in 2d space
    """
    return np.sqrt((q[0] - p[0]) ** 2 + (q[1] - p[1]) ** 2)

def find_correspondences(past_pcl, curr_pcl):
    # TODO: correspondences 1D

    N_curr = curr_pcl.shape[0]
    N_past = past_pcl.shape[0]

    N_smaller = min(N_curr, N_past)

    N = 2
    correspondences = np.zeros((2,N_smaller-(2*N)+1), dtype=int)

    iteration = 0
    for i in range(N, N_smaller - N+1):
        past_point = past_pcl[i]
        corr_slice = curr_pcl[i-N:i+N]
        distances = [distance(past_point, curr_p) for curr_p in corr_slice]
        corr_index = np.argmin(distances)+iteration
        correspondences[0,iteration] = corr_index
        iteration += 1

    correspondences[1,:] = correspondences[0,:]
    return correspondences

#past_pcl = np.array([[1.2, 1.2], [0, 0], [1, 1.4], [2, 2], [3.2, 3.6], [4.1, 4.0], [5.3, 5.1], [6.1, 6.0]])
#curr_pcl = np.array([[0, 0], [1.7, 1], [2.5, 2], [3.3, 3], [4.2, 3.9], [5.3, 5.1], [6.1, 5.9], [7.1, 6.9]])

pose = np.array([[0.0], [0.0]])

for i in range(-1, 11):
    N = 1000
    a = np.linspace(i, i+100, N)
    b = np.linspace(i+1, i+101, N)
    noise = np.random.uniform(low=-0.2, high=0.3, size=(N,))
    a += noise
    b += noise

    past_pcl = np.stack((a, a+noise), axis=-1)
    curr_pcl = np.stack((b+noise, b), axis=-1)
    correspondences = find_correspondences(past_pcl, curr_pcl)
    trans, rot = find_transformation(past_pcl, curr_pcl, correspondences)
    pose += trans

    print "\n--- iteration", i, "---"
    print "translation x:", np.round(trans[0,0], 3), "y:", np.round(trans[1,0], 3)
    print "rotation:", rot, "degrees" 
    print "location:", pose[0,0],"/", pose[1,0]