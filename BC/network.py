import torch
import torch.nn as nn


LONG_STD_MIN = -20
LONG_STD_MAX = 2

class BCND_network(nn.Module):
    def __init__(self,
                 in_dim, 
                 out_dim,
                 hidden_dim=100, 
                 num_hidden_layers=2) -> None:
        super(BCND_network,self).__init__()
        self.in_dim = in_dim
        self.hidden_dim = hidden_dim
        self.num_hidden_layers = num_hidden_layers
        self.out_dim = out_dim
        # building network
        self.layers = []
        self.layers.append(nn.Linear(in_dim, hidden_dim))
        for _ in range(self.num_hidden_layers):
            self.layers.append(nn.Linear(self.hidden_dim, self.hidden_dim))
            self.layers.append(nn.Tanh())
        self.network = nn.Sequential(*self.layers)
        self.mean = nn.Linear(hidden_dim, out_dim)
        self.long_std = nn.Linear(hidden_dim,out_dim)
        # xavier initialization
        for m in self.network.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_uniform_(m.weight)
        nn.init.xavier_uniform_(self.mean.weight)
        nn.init.xavier_uniform_(self.long_std.weight)

    def forward(self, data:torch.Tensor):
        # make a batch if data is a single vector
        if len(data.size()) == 1:
            data.unsqueeze(0)
        # print("data:{}\n".format(data))
        x = self.network(data)
        # print("x:{}\n".format(x))
        mean = self.mean(x)
        long_std = self.long_std(x)
        long_std = torch.clamp(long_std, min=LONG_STD_MIN,max=LONG_STD_MAX)
        std = long_std.exp()
        return mean,std
    
    def train(self):
        super().train()
    def eval(self):
        super().eval()    

