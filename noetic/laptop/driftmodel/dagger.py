#! /usr/bin/env python

import argparse
import numpy as np
import pickle
import os
from datetime import datetime
import gzip
import json
from model import *
import train_agent
from utils import *
import torch
import pandas as pd

import signal

import time

from driftmodel_env import DriftEnv

global shut
shut = False

NUM_ITS = 20
beta_i  = 0.9

def store_data(data, datasets_dir):
    # save data
    if not os.path.exists(datasets_dir):
        os.mkdir(datasets_dir)
    data_file = os.path.join(datasets_dir, 'data_dagger.pkl.gzip')
    f = gzip.open(data_file,'wb')
    pickle.dump(data, f)


def save_results(episode_rewards, results_dir):
    # save results
    if not os.path.exists(results_dir):
        os.mkdir(results_dir)

     # save statistics in a dictionary and write them into a .json file
    results = dict()
    results["number_episodes"] = len(episode_rewards)
    results["episode_rewards"] = episode_rewards

    results["mean_all_episodes"] = np.array(episode_rewards).mean()
    results["std_all_episodes"] = np.array(episode_rewards).std()
 
    fname = os.path.join(results_dir, "results_manually-%s.json" % datetime.now().strftime("%Y%m%d-%H%M%S"))
    fh = open(fname, "w")
    json.dump(results, fh)
    print('... finished')

'''def save_data_to_csv(data, file_path="./csv_data"):
    if not os.path.exists(file_path):
        os.mkdir(file_path)
        data.to_csv(file_path, index=False)
    else:    
        existing_data = pd.read_csv(file_path)
        combined_data = pd.concat([existing_data, data], ignore_index=True)
        combined_data.to_csv(file_path, index=False)
        data.to_csv(file_path, index=False)
'''

def SignalHandler_SIGINT(SignalNumber,Frame):
     global shut
     shut = True
     print('SignalHandler of signal.SIGINT')

#register the signal with Signal handler
signal.signal(signal.SIGINT,SignalHandler_SIGINT)
    
if __name__ == "__main__":

    if not os.path.exists("/home/ubuntu/ros_noetic_ws/src/driftmodel/src/dagger_models"):
        os.mkdir("/home/ubuntu/ros_noetic_ws/src/driftmodel/src/dagger_models")
 

    samples = {
      "state": [],
      "next_state": [],
      "reward": [],
      "action": [],
      "terminal" : [],
    }
        
    #time.sleep(10)#38

    env = DriftEnv()

    env.reset()
    
    print("initialized!")
    
    episode_rewards = []
    steps = 0
    agent = Model()
    agent.save("/home/ubuntu/ros_noetic_ws/src/driftmodel/src/dagger_models/model_4.pth")
    
    model_number = 3
    old_model_number = 3
    done_ = False
    
    print("saved model!")
    
    while(done_ == False and shut == False):
       
        agent.load("/home/ubuntu/ros_noetic_ws/src/driftmodel/src/dagger_models/model_{}.pth".format(model_number))
        curr_beta = beta_i ** model_number 

        if model_number != old_model_number:
            print("Using model : {}".format(model_number))
            print("Beta value: {}".format(curr_beta))
            old_model_number = model_number

        episode_reward = 0
        state = env.reset() 
        # pi: input to the environment
        # a : expert input
        pi_s = np.array([0.0]).astype('float32')
        pi_t = np.array([0.0, 0.0]).astype('float32')
        pi = np.array([pi_s[0], pi_t[0], pi_t[1]]).astype('float32')
        a = np.zeros_like(pi)
        
        while True:

            next_state, r, done, info = env.step(pi)
            a = env.actionCB()
            print("your action: ", a)
            
            done_ = done

            prediction = agent(torch.from_numpy(next_state).type(torch.FloatTensor))
            
            print("steer predicted: ", prediction[0])
            print("throttle predicted: ", prediction[1:])
            print("policy: ", pi)
            #print("shut: ", shut)
            # calculate linear combination of expert and network policy
            #pi = curr_beta * a + (1 - curr_beta) * prediction.detach().numpy().flatten()
            
            pi = curr_beta * a + (1 - curr_beta) * prediction.detach().numpy()
            episode_reward += r
            
            #print(r)
            print("summation: ", episode_reward)

            samples["state"].append(state)           
            samples["action"].append(np.array(a))     
            samples["next_state"].append(next_state)
            samples["reward"].append(r)
            samples["terminal"].append(done)
                        
            state = next_state
            steps += 1
           
            if(shut==True):
                break 

            # train after T steps
            if done_ == True:
                time.sleep(3)
                print('... saving data')
                store_data(samples,"/home/ubuntu/ros_noetic_ws/src/driftmodel/src/data")
                save_results(episode_rewards, "/home/ubuntu/ros_noetic_ws/src/driftmodel/src/results")

                '''data_ = pd.DataFrame(samples)
                save_data_to_csv(data_, "./csv_data")'''
                X_train, y_train, X_valid, y_valid = train_agent.read_data("/home/ubuntu/ros_noetic_ws/src/driftmodel/src/data", "data_dagger.pkl.gzip")
                train_agent.train_model(X_train, y_train, X_valid, y_valid, "/home/ubuntu/ros_noetic_ws/src/driftmodel/src/dagger_models/model_{}.pth".format(model_number+1), num_epochs=10)

                model_number += 1
                print("Training complete. Press return to continue to the next iteration")
                done_ = False
                break       
        episode_rewards.append(episode_reward)
            
    env.sig_shut()

