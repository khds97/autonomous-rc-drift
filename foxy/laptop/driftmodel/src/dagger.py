#!/usr/bin/env python3
"""DAgger data collection and policy training loop.

Every episode the operator drives (and, from the second iteration on, the current policy is mixed in with
the weight 1 - beta, beta = 0.9 ** model_number). The states and the operator actions are appended to the
dataset, then the next model is trained on all data collected so far.

Model 0 means "no model yet": beta is 1 and the commands are the operator's own.
"""

import argparse
import glob
import gzip
import json
import os
import pickle
import re
import signal
import sys
from datetime import datetime

import numpy as np
import torch

import train_agent
from model import Model
from preprocess import FEATURE_NAMES, Preprocessor

BETA_BASE = 0.9
MAX_CONSECUTIVE_TIMEOUTS = 20
MIN_SAMPLES_TO_TRAIN = 64
DATA_FILE = 'data_dagger.pkl.gzip'
DEFAULT_ROOT = os.path.join(os.path.expanduser('~'), 'driftmodel')

KEYS = ('state', 'next_state', 'reward', 'action', 'terminal', 'mode_flag')


class DataStore:
    """The recorded dataset. It is loaded at start and appended to, so DAgger keeps aggregating
    across runs."""

    def __init__(self, data_dir, filename=DATA_FILE):
        self.data_dir = data_dir
        self.filename = filename
        self.path = os.path.join(data_dir, filename)
        self.samples = {k: [] for k in KEYS}

        if os.path.exists(self.path):
            with gzip.open(self.path, 'rb') as f:
                loaded = pickle.load(f)
            n = len(loaded['state'])
            for k in KEYS:
                if k in loaded:
                    self.samples[k] = list(loaded[k])
                elif k == 'mode_flag':
                    self.samples[k] = [False] * n
                else:
                    raise KeyError("the dataset {} has no '{}' entry".format(self.path, k))
            if n and len(np.asarray(self.samples['action'][0]).ravel()) != 2:
                raise ValueError("the dataset {} has 3 action values (steer, forward, brake); "
                                 "expected 2 (steer, throttle). Record a new dataset".format(self.path))

    def extend(self, episode):
        for k in KEYS:
            self.samples[k].extend(episode[k])

    def save(self):
        os.makedirs(self.data_dir, exist_ok=True)
        tmp = self.path + '.tmp'
        with gzip.open(tmp, 'wb') as f:
            pickle.dump(self.samples, f)
        os.replace(tmp, self.path)         # the old file stays intact if writing fails

    def __len__(self):
        return len(self.samples['state'])


def model_path(model_dir, number):
    return os.path.join(model_dir, 'model_{}.pth'.format(number))


def latest_model_number(model_dir):
    """Highest N for which model_N.pth and its scaler both exist; 0 when there is none."""
    best = 0
    for path in glob.glob(os.path.join(model_dir, 'model_*.pth')):
        match = re.match(r'model_(\d+)\.pth$', os.path.basename(path))
        if match and Preprocessor.exists(path):
            best = max(best, int(match.group(1)))
    return best


def load_policy(model_dir, number):
    path = model_path(model_dir, number)
    prep = Preprocessor.load(path)
    agent = Model(prep.n_in, prep.n_out)
    agent.load(path)
    agent.eval()
    return agent, prep


def save_results(episode_rewards, results_dir):
    os.makedirs(results_dir, exist_ok=True)
    rewards = [float(r) for r in episode_rewards]
    results = {
        "number_episodes": len(rewards),
        "episode_rewards": rewards,
        "mean_all_episodes": float(np.mean(rewards)) if rewards else None,
        "std_all_episodes": float(np.std(rewards)) if rewards else None,
    }
    fname = os.path.join(results_dir, "results_manually-%s.json" % datetime.now().strftime("%Y%m%d-%H%M%S"))
    with open(fname, "w") as fh:
        json.dump(results, fh)
    return fname


def run_dagger(env, store, model_dir, results_dir, beta_base=BETA_BASE, epochs=10,
               input_features=None, exclude_autonomous=False, should_stop=lambda: False, log=print):
    """Runs the DAgger loop until should_stop() returns True. `env` needs reset(), step() and send_neutral()."""
    os.makedirs(model_dir, exist_ok=True)

    model_number = latest_model_number(model_dir)
    episode_rewards = []

    while not should_stop():
        beta = beta_base ** model_number
        agent = prep = None
        if model_number > 0:
            agent, prep = load_policy(model_dir, model_number)
        log("Using model: {}  (beta = {:.3f}, {} samples in the dataset)".format(model_number, beta, len(store)))

        sample = env.reset()
        episode = {k: [] for k in KEYS}
        episode_reward = 0.0
        done = False
        timeouts = 0

        try:
            while not done and not should_stop():
                expert = np.asarray(sample.expert_action, dtype=np.float64)

                if agent is None:
                    pi = expert
                else:
                    with torch.no_grad():
                        out = agent(torch.from_numpy(prep.transform_x(sample.state))).numpy()
                    # both the expert action and the prediction are in volts, so they can be mixed directly
                    pi = beta * expert + (1.0 - beta) * prep.inverse_y(out)

                next_sample, reward, done, info = env.step(pi)

                if info.get('timeout'):
                    timeouts += 1
                    if timeouts >= MAX_CONSECUTIVE_TIMEOUTS:
                        raise RuntimeError("no new sensor data for {} steps".format(timeouts))
                    continue
                timeouts = 0

                # the state and the label come from the same synchronized sample
                episode['state'].append(sample.state)
                episode['action'].append(np.asarray(sample.expert_action, dtype=np.float32))
                episode['next_state'].append(next_sample.state)
                episode['reward'].append(reward)
                episode['terminal'].append(bool(done))
                episode['mode_flag'].append(bool(sample.mode_flag))
                episode_reward += reward
                sample = next_sample
        finally:
            env.send_neutral()      # never leave the last command active
            if episode['state']:
                episode['terminal'][-1] = True       # the last sample of an episode is always terminal
                store.extend(episode)
                store.save()
                log("... saved {} samples (dataset: {})".format(len(episode['state']), len(store)))

        episode_rewards.append(episode_reward)
        save_results(episode_rewards, results_dir)

        if should_stop():
            break

        if len(store) < MIN_SAMPLES_TO_TRAIN:
            log("... only {} samples so far, collecting more before training".format(len(store)))
            continue

        next_number = model_number + 1
        X_train, y_train, X_valid, y_valid, new_prep = train_agent.read_data(
            store.data_dir, store.filename, input_features=input_features,
            exclude_autonomous=exclude_autonomous)
        train_agent.train_model(X_train, y_train, X_valid, y_valid, model_path(model_dir, next_number),
                                new_prep, num_epochs=epochs)
        model_number = next_number
        log("Training complete: model_{}.pth".format(model_number))

    return len(episode_rewards)


def main(argv=None):
    import rclpy
    from rclpy.utilities import remove_ros_args
    from driftmodel_env import DriftEnv

    if argv is None:
        argv = remove_ros_args(args=sys.argv)[1:]

    parser = argparse.ArgumentParser()
    parser.add_argument('--data_dir', default=os.path.join(DEFAULT_ROOT, 'data'))
    parser.add_argument('--model_dir', default=os.path.join(DEFAULT_ROOT, 'dagger_models'))
    parser.add_argument('--results_dir', default=os.path.join(DEFAULT_ROOT, 'results'))
    parser.add_argument('--serial_port', default='/dev/ACMSTM32')
    parser.add_argument('--beta', type=float, default=BETA_BASE, help='beta of iteration n is beta ** n')
    parser.add_argument('--epochs', type=int, default=10)
    parser.add_argument('--use_tire_pwm', action='store_true',
                        help='also give tire_pwm to the network (see preprocess.py)')
    parser.add_argument('--exclude_autonomous', action='store_true',
                        help='do not train on samples recorded while the laptop was driving')
    args = parser.parse_args(argv)

    rclpy.init()
    shut = {'flag': False}

    def on_sigint(signum, frame):
        shut['flag'] = True
        print('SIGINT received, finishing the current step')

    signal.signal(signal.SIGINT, on_sigint)

    env = DriftEnv(serial_port=args.serial_port)
    env.spin_in_background()
    print("initialized!")

    try:
        store = DataStore(args.data_dir)
        run_dagger(env, store, args.model_dir, args.results_dir, beta_base=args.beta, epochs=args.epochs,
                   input_features=list(FEATURE_NAMES) if args.use_tire_pwm else None,
                   exclude_autonomous=args.exclude_autonomous,
                   should_stop=lambda: shut['flag'] or not rclpy.ok())
    finally:
        env.send_neutral()
        env.close()
        env.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
