import torch
import torch.nn as nn
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from replay_buffer import SimpleReplayBuffer
from utils.bc_utils import *
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

state_space_size = 7
action_space_size = 7
bc_walker  =  nn.Sequential(
        nn.Linear(state_space_size,40),
        nn.ReLU(),
        
        nn.Linear(40,80),
        nn.ReLU(),
        
        nn.Linear(80,120),
        nn.ReLU(),
        
        nn.Linear(120,100),
        nn.ReLU(),
        
        nn.Linear(100,40),
        nn.ReLU(),
        
        nn.Linear(40,20),
        nn.ReLU(),
        
        
        nn.Linear(20,action_space_size),
    ).to(device)
class rollout_vanilla_BC():
    def __init__(self):
        self.policies = bc_walker
        self.model_path = "/home/hang/catkin_ws/src/ldf_with_progress/BC_vanilla/vanilla_model.pth"
    def load_model(self):
        self.policies.load_state_dict(torch.load(self.model_path))
    def get_action(self,state):
        state = torch.tensor(state).float().to(device)
        state = state.unsqueeze(0)
        action = self.policies(state)
        return action.detach().cpu().numpy()
    

if __name__ == "__main__":
    

    buffer  =SimpleReplayBuffer(state_space_size,action_space_size,10000)

    cup_idx_list = read_cup_index_from_csv("/home/hang/catkin_ws/src/ldf_with_progress/BC/participant_sheet.csv")
    #         # spilt_traj_with_cup_pos_and_read_to_buffer(self.trainer.buffers, msgs, cup_idx_list,0)
    # txt_list = find_all_txt_files_with_eef()
    # msgs = read_txt_to_meaasages(txt_list)
    # split_traj_with_cup_pos_and_read_eef(buffer, msgs, cup_idx_list,2)

    bagfile_list = get_all_bag_files()
    msg_list = whole_bag_to_messages(bagfile_list)
    spilt_traj_with_cup_pos_and_read_to_buffer(buffer, msg_list, cup_idx_list,2)
    print(buffer.size)


    
    criterion = nn.MSELoss()
    learning_rate = 0.01
    optimizer = torch.optim.Adam(bc_walker.parameters(), lr = learning_rate) 
    loss_list = []
    test_loss = []
    batch_size = 256
    n_epoch = 100
    learning_rate = 0.001
    optimizer = torch.optim.Adam(bc_walker.parameters(), lr = learning_rate) 
    for itr in range(n_epoch):
        total_loss = 0
        b=0
        for batch in range (0,buffer.size-200, batch_size):
            data   = buffer.observations[batch : batch+batch_size ]
            y      = buffer.actions[batch : batch+batch_size]
            data  = torch.tensor(data).float().to(device)
            y     = torch.tensor(y).float().to(device)
            y_pred = bc_walker(data)
            loss   = criterion(y_pred, y)
            total_loss += loss.item() 
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            b += 1
        print("[EPOCH]: %i, [MSE LOSS]: %.6f" % (itr+1, total_loss / b))
        # display.clear_output(wait=True)
        loss_list.append(total_loss / b)
        x = buffer.observations[buffer.size-200:]
        y = buffer.actions[buffer.size-200:]
        x = torch.tensor(x).float().to(device)
        y = torch.tensor(y).float().to(device)  
        y_pred = bc_walker(x)
        test_loss.append(criterion(y_pred, y).item())
    torch.save(bc_walker.state_dict(), "/home/hang/catkin_ws/src/ldf_with_progress/BC_vanilla/vanilla_model.pth")
    print(test_loss)