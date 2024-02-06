from BC.utils.save_model import load_model
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import torch
PATH = os.path.dirname(os.path.abspath(__file__))
ACTION_DIM = 6
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class roll_out_BCND():
    def __init__(self):
        pass
    def load_model(self, model_path):
        self.policies = load_model(model_path)
    def get_action(self,state):
        state = torch.tensor(state).float().to(DEVICE)
        state = state.unsqueeze(0)
        actions = []
        for i in range(len(self.policies)):
            # self.policies[i].eval()
            action_i_mean, action_i_std = self.policies[i](state)
            action_i_distribution = torch.distributions.normal.Normal(action_i_mean, action_i_std)
            action_i = action_i_distribution.sample()
            actions.append(action_i)
        print("action_tensor:{}\n".format(actions))
        action = torch.stack(actions).mean(dim=0,keepdim=False).detach().cpu().numpy()
        return action


    