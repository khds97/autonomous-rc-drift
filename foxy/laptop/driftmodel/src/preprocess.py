import os
import joblib
import numpy as np
from sklearn.preprocessing import StandardScaler

# Order of the 21-dim state that is recorded in the dataset. Must match DriftEnv.
FEATURE_NAMES = ['px', 'py', 'pz',
                 'qw', 'qx', 'qy', 'qz',
                 'vx', 'vy', 'vz',
                 'ax', 'ay', 'az',
                 'gx', 'gy', 'gz',
                 'rpm', 'tire_pwm', 'rl_speed', 'rr_speed', 'current']

# Order of the 2-dim action: operator steering and throttle (voltage, 0 .. 3.3 V).
ACTION_NAMES = ['steer', 'throttle']

# State features that are recorded but not given to the network by default.
#  - tire_pwm : strongly correlated with the steering label (-0.64) in the first dataset.
#               Without it the steering error went from 0.069 to 0.167 in a 5-fold temporal
#               cross-validation. If this pin carries the steering signal that is applied to
#               the car, it leaks the action into the state.
#               It is still recorded; use `--use_tire_pwm` to feed it to the network.
DEFAULT_EXCLUDED_FEATURES = ('tire_pwm',)

# Input features that are standardized with StandardScaler.
#  - px, py       : wide range.
#  - pz           : NOT scaled. Small spread (about 0.09 in the first dataset); scaling would
#                   amplify small fluctuations. The physical meaning of this axis was not checked.
#  - qw,qx,qy,qz  : NOT scaled. Quaternion components are already bounded to [-1, 1];
#                   qw and qz have a std of about 0.007 which scaling would amplify ~100x.
#  - everything else (velocity, IMU, wheel and drive sensors): different units and ranges.
SCALE_FEATURES = ['px', 'py',
                  'vx', 'vy', 'vz',
                  'ax', 'ay', 'az',
                  'gx', 'gy', 'gz',
                  'rpm', 'tire_pwm', 'rl_speed', 'rr_speed', 'current']


def default_input_features():
    return [n for n in FEATURE_NAMES if n not in DEFAULT_EXCLUDED_FEATURES]


def scaler_path(model_path):
    """model_3.pth -> model_3_scaler.pkl (each model is trained on different data, so each has its own scaler)."""
    root, _ = os.path.splitext(model_path)
    return root + '_scaler.pkl'


class Preprocessor:
    """
    state  -> network input : selects `input_features` and standardizes those in SCALE_FEATURES.
    action -> network target: all values are standardized; inverse_y() restores the original units.

    Fit on the training split only, and save/load it together with the model it was used for.
    The selected features are stored in the object, so inference always uses the same inputs as training.
    """

    def __init__(self, input_features=None):
        if input_features is None:
            input_features = default_input_features()
        unknown = [n for n in input_features if n not in FEATURE_NAMES]
        if unknown:
            raise ValueError('unknown feature name(s): {}'.format(unknown))
        self.input_features = list(input_features)
        self.input_idx = [FEATURE_NAMES.index(n) for n in self.input_features]
        # positions (inside the network input) of the features that are standardized
        self.scale_cols = [j for j, n in enumerate(self.input_features) if n in SCALE_FEATURES]
        self.x_scaler = StandardScaler()
        self.y_scaler = StandardScaler()

    @property
    def n_in(self):
        return len(self.input_features)

    @property
    def n_out(self):
        return len(ACTION_NAMES)

    def _check_state(self, X):
        if X.shape[1] != len(FEATURE_NAMES):
            raise ValueError('expected {} state columns, got {}'.format(len(FEATURE_NAMES), X.shape[1]))

    def _check_action(self, y):
        if y.shape[1] != len(ACTION_NAMES):
            raise ValueError('expected {} action columns ({}), got {}'.format(
                len(ACTION_NAMES), ', '.join(ACTION_NAMES), y.shape[1]))

    def fit(self, X, y):
        X = np.asarray(X, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        self._check_state(X)
        self._check_action(y)
        selected = X[:, self.input_idx]
        if self.scale_cols:
            self.x_scaler.fit(selected[:, self.scale_cols])
        self.y_scaler.fit(y)
        return self

    def transform_x(self, X):
        """(21,) or (N, 21) full state -> (n_in,) or (N, n_in) network input, float32."""
        X = np.asarray(X, dtype=np.float32)
        single = X.ndim == 1
        X = np.atleast_2d(X)
        self._check_state(X)
        out = X[:, self.input_idx].copy()
        if self.scale_cols:
            out[:, self.scale_cols] = self.x_scaler.transform(out[:, self.scale_cols].astype(np.float64))
        out = out.astype(np.float32)
        return out[0] if single else out

    def transform_y(self, y):
        """Original-unit action -> normalized action. (2,) or (N, 2)."""
        y = np.asarray(y, dtype=np.float64)
        return self.y_scaler.transform(np.atleast_2d(y)).astype(np.float32).reshape(y.shape)

    def inverse_y(self, y):
        """Network output (normalized action) -> original-unit action. (2,) or (N, 2)."""
        y = np.asarray(y, dtype=np.float64)
        return self.y_scaler.inverse_transform(np.atleast_2d(y)).astype(np.float32).reshape(y.shape)

    def save(self, model_path):
        joblib.dump(self, scaler_path(model_path))

    @staticmethod
    def exists(model_path):
        return os.path.exists(scaler_path(model_path))

    @staticmethod
    def load(model_path):
        path = scaler_path(model_path)
        if not os.path.exists(path):
            raise FileNotFoundError(
                'Scaler file not found: {}\n'
                'A model is only usable together with the scaler it was trained with. '
                'Train it again with:  ros2 run driftmodel train_agent.py {}'.format(path, model_path))
        return joblib.load(path)
