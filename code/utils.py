
import pickle
import numpy as np


def saveObject(tau, fname):
    with open(fname, 'wb') as f:
        pickle.dump(tau, f)


def loadObject(fname):
    with open(fname, 'rb') as f:
        tau = pickle.load(f)
    return tau


def saveNetworks(nn, nnQ, fname):
    with open(fname, 'wb') as f:
        pickle.dump({'nn': nn, 'nnQ': nnQ}, f)


def loadNetworks(fname):
    with open(fname, 'rb') as f:
        datdict = pickle.load(f)
    return datdict['nn'], datdict['nnQ']


def add_dim_last(obs):
    return np.expand_dims(obs, 1)
