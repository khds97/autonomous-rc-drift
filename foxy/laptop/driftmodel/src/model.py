import torch
import torch.nn as nn


class Model(nn.Module):
    def __init__(self, in_dim=20, out_dim=2):
        """
        in_dim  : number of state features fed to the network (Preprocessor.n_in)
        out_dim : number of action values (steer, throttle)
        """
        super(Model, self).__init__()

        self.lin1 = nn.Linear(in_dim, 11)
        self.lin2 = nn.Linear(11, 5)
        self.lin3 = nn.Linear(5, out_dim)

        self.relu = nn.ReLU()

    def forward(self, x):

        x = self.lin1(x)
        x = self.relu(x)
        x = self.lin2(x)
        x = self.relu(x)
        x = self.lin3(x)

        return x

    def load(self, path):
        self.load_state_dict(torch.load(path, map_location='cpu'))

    def save(self, path):
        torch.save(self.state_dict(), path)
