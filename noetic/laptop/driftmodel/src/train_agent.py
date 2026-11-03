from __future__ import print_function
import argparse
import pickle
import numpy as np
import os
import gzip
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler

from model import *
from utils import *
import torch

def read_data(datasets_dir="./data", path='data.pkl.gzip', mode="none", frac = 0.1):
    """
    This method reads the states and actions recorded in drive_manually.py 
    and splits it into training/ validation set.
    """
    print("... read data")
    data_file = os.path.join(datasets_dir, path)
  
    f = gzip.open(data_file,'rb')
    data = pickle.load(f)

    # get images as features and actions as targets
    X = np.array(data["state"]).astype('float32')
    y = np.array(data["action"]).astype('float32')
        
    # split data into training and validation set
    n_samples = len(data["state"])
    X_train, y_train = X[:int((1-frac) * n_samples)], y[:int((1-frac) * n_samples)]
    X_valid, y_valid = X[int((1-frac) * n_samples):], y[int((1-frac) * n_samples):]
    
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_valid_scaled = scaler.transform(X_valid)
    #X = scaler.transform(X)
    
    #print("x: ", X_train[int(n_samples * frac)])
    #print("y: ", y_train[int(n_samples * frac)])
    
    #print(X_train.size)

    
    return X_train_scaled, y_train, X_valid_scaled, y_valid 

def train_model(X_train, y_train, X_valid, y_valid, path, num_epochs=50, learning_rate=1e-3, lambda_l2=1e-5, batch_size=32):

    model = Model()
    criterion = torch.nn.MSELoss()
    optimizer = torch.optim.SGD(model.parameters(), lr=learning_rate, weight_decay=lambda_l2) # built-in L2 

    #X_train_torch = torch.from_numpy(X_train[:,np.newaxis,...]) when the data has images
    X_train_torch = torch.from_numpy(X_train[:])
    y_train_torch = torch.from_numpy(y_train)
    for t in range(num_epochs):
      print("[EPOCH]: %i" % (t), end='\r')
      for i in range(0,len(X_train_torch),batch_size):
        acc = 0
        curr_X = X_train_torch[i:i+batch_size]
        curr_Y = y_train_torch[i:i+batch_size]
        preds  = model(curr_X)
        #if(i==(batch_size * 4)):
        #	print("predicted: ", preds)
        #	print("label    : ", curr_Y)
        loss   = criterion(preds, curr_Y)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    model.save(path)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('model_name', metavar='M', default='model.pth', type=str, help='model name to save')
    
    args = parser.parse_args()

    # read data
    X_train, y_train, X_valid, y_valid = read_data("./data", frac=0.9)

    # train models based on command-line arguments
 
    train_model(X_train, y_train, X_valid, y_valid, args.model_name, num_epochs=10)


