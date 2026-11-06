#!/usr/bin/env python3
from __future__ import print_function
import argparse
import gzip
import os
import pickle

import numpy as np
import torch

from model import Model
from preprocess import ACTION_NAMES, FEATURE_NAMES, Preprocessor


def read_data(datasets_dir="./data", path='data_dagger.pkl.gzip', frac=0.1,
              input_features=None, exclude_autonomous=False):
    """
    Reads the recorded states and actions and splits them into a training and a validation set
    (the last `frac` of the samples, in recording order, is used for validation).

    The Preprocessor is fitted on the training split only. Returns the normalized splits and the
    fitted Preprocessor, which is needed to normalize states and de-normalize actions at inference.

    input_features     : state features given to the network (default: all except tire_pwm)
    exclude_autonomous : drop the samples that were recorded while the laptop was driving
                         (use this if the expert label is not reliable in that mode)
    """
    print("... read data")
    data_file = os.path.join(datasets_dir, path)

    with gzip.open(data_file, 'rb') as f:
        data = pickle.load(f)

    X = np.array(data["state"]).astype('float32')
    y = np.array(data["action"]).astype('float32')

    if X.ndim != 2 or X.shape[1] != len(FEATURE_NAMES):
        raise ValueError("expected a state with {} columns, found shape {}".format(len(FEATURE_NAMES), X.shape))
    if y.ndim != 2 or y.shape[1] != len(ACTION_NAMES):
        raise ValueError(
            "expected an action with {} columns ({}), found shape {}. A dataset with 3 action "
            "columns (steer, forward, brake) has to be recorded again.".format(
                len(ACTION_NAMES), ", ".join(ACTION_NAMES), y.shape))

    if exclude_autonomous and "mode_flag" in data:
        keep = ~np.array(data["mode_flag"]).astype(bool)
        X, y = X[keep], y[keep]
        print("... excluded the autonomous-mode samples, {} samples left".format(len(X)))

    n_samples = len(X)
    n_train = int((1 - frac) * n_samples)
    X_train, y_train = X[:n_train], y[:n_train]
    X_valid, y_valid = X[n_train:], y[n_train:]

    # fit on the training split only (no leakage from the validation split)
    prep = Preprocessor(input_features).fit(X_train, y_train)

    X_train_scaled = prep.transform_x(X_train)
    X_valid_scaled = prep.transform_x(X_valid)
    y_train_scaled = prep.transform_y(y_train)
    y_valid_scaled = prep.transform_y(y_valid)

    return X_train_scaled, y_train_scaled, X_valid_scaled, y_valid_scaled, prep


def evaluate(model, X_valid, y_valid, prep):
    """Validation MSE in the original units, next to two baselines:
    the training mean, and "repeat the previous action" (the actions are very smooth in time)."""
    with torch.no_grad():
        pred = model(torch.from_numpy(X_valid)).numpy()
    pred_raw = prep.inverse_y(pred)
    y_raw = prep.inverse_y(y_valid)

    mse = np.mean((pred_raw - y_raw) ** 2, axis=0)
    mean_mse = np.mean((y_raw - prep.y_scaler.mean_) ** 2, axis=0)
    if len(y_raw) > 1:
        prev_mse = np.mean((y_raw[1:] - y_raw[:-1]) ** 2, axis=0)   # ignores episode boundaries
    else:
        prev_mse = np.full(len(ACTION_NAMES), np.nan)
    return {"model": mse, "mean": mean_mse, "previous": prev_mse}


def train_model(X_train, y_train, X_valid, y_valid, path, prep, num_epochs=50, learning_rate=1e-3,
                lambda_l2=1e-5, batch_size=32, seed=None):
    """
    y_* are expected to be normalized actions (as returned by read_data).
    The model (path) and its Preprocessor (<path without .pth>_scaler.pkl) are saved together.
    Returns the validation metrics (see evaluate) or None when there is no validation data.
    """
    if seed is not None:
        torch.manual_seed(seed)

    model = Model(prep.n_in, prep.n_out)
    criterion = torch.nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate, weight_decay=lambda_l2)

    X_train_torch = torch.from_numpy(X_train)
    y_train_torch = torch.from_numpy(y_train)
    n = len(X_train_torch)

    for t in range(num_epochs):
        print("[EPOCH]: %i" % (t), end='\r')
        order = torch.randperm(n)          # shuffle every epoch
        for i in range(0, n, batch_size):
            idx = order[i:i + batch_size]
            loss = criterion(model(X_train_torch[idx]), y_train_torch[idx])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
    print()

    metrics = None
    if len(X_valid) > 0:
        metrics = evaluate(model, X_valid, y_valid, prep)
        print("[VALID] MSE %s: %s" % ("/".join(ACTION_NAMES), " / ".join("%.4f" % v for v in metrics["model"])))
        print("        training-mean baseline:  %s" % " / ".join("%.4f" % v for v in metrics["mean"]))
        print("        repeat-previous-action:  %s" % " / ".join("%.4f" % v for v in metrics["previous"]))

    model.save(path)
    prep.save(path)
    return metrics


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('model_name', metavar='M', type=str, help='model file to save, e.g. model_1.pth')
    parser.add_argument('--data_dir', default='./data', type=str, help='directory of the recorded data')
    parser.add_argument('--data_file', default='data_dagger.pkl.gzip', type=str, help='recorded data file name')
    parser.add_argument('--epochs', default=10, type=int)
    parser.add_argument('--use_tire_pwm', action='store_true',
                        help='also give tire_pwm to the network (see DEFAULT_EXCLUDED_FEATURES in preprocess.py)')
    parser.add_argument('--exclude_autonomous', action='store_true',
                        help='drop the samples recorded while the laptop was driving')
    parser.add_argument('--seed', default=None, type=int)
    args = parser.parse_args(argv)

    input_features = list(FEATURE_NAMES) if args.use_tire_pwm else None

    # read data (last 10% of the samples are used for validation)
    X_train, y_train, X_valid, y_valid, prep = read_data(
        args.data_dir, args.data_file, frac=0.1,
        input_features=input_features, exclude_autonomous=args.exclude_autonomous)

    train_model(X_train, y_train, X_valid, y_valid, args.model_name, prep,
                num_epochs=args.epochs, seed=args.seed)


if __name__ == "__main__":
    main()
