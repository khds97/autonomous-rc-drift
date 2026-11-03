import torch
import torch.nn as nn

class Model(nn.Module):
    def __init__(self):
        super(Model, self).__init__()
        
        self.lin1  = nn.Linear(21, 11)
        self.lin2  = nn.Linear(11 ,5)
        self.lin3  = nn.Linear(5 ,3)

        self.relu  = nn.ReLU()

    def forward(self, x):
        
        x = self.lin1(x)
        x = self.relu(x)
        x = self.lin2(x)
        x = self.relu(x)   
        x = self.lin3(x)
       
        return x
        
    def load(self, path):
        self.load_state_dict(torch.load(path))

    def save(self, path):
        torch.save(self.state_dict(), path)
