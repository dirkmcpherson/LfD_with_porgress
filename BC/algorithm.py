from agent import BCND_Trainer
from utils.bc_utils import *
from utils.save_model import save_model
import os



PATH = os.path.dirname(os.path.abspath(__file__))

class BCND_algorithm():
    def __init__(self, 
                 iterartion_num,
                obs_dim,
            action_dim,            
            max_buffer_size,
            batch_size,
            training_horizon,
            learning_rate:float,
            num_networks:int,
            network_config:dict,
            eval_freq:int):
        
        self.trainer = BCND_Trainer(obs_dim,
            action_dim,            
            max_buffer_size,
            batch_size,
            training_horizon,
            learning_rate,
            num_networks,
            network_config)
        self.eval_freq = eval_freq
        self.iteration_num = iterartion_num
        # load trajectory data to buffer
        ls = get_all_bag_files()
        # msgs = whole_bag_to_messages(ls)
        # read_one_trajectory_to_each_buffer(num_networks,self.trainer.buffers, msgs)
        cup_idx_list = read_cup_index_from_csv("/home/hang/catkin_ws/src/ldf_with_progress/BC/participant_sheet.csv")
        # spilt_traj_with_cup_pos_and_read_to_buffer(self.trainer.buffers, msgs, cup_idx_list,0)
        txt_list = find_all_txt_files_with_eef()
        msgs = read_txt_to_meaasages(txt_list)
        split_traj_with_cup_pos_and_read_eef(self.trainer.buffers, msgs, cup_idx_list,0)

    def train(self):
        for i in range(self.iteration_num):
            self.trainer.run_one_iterarion()
            if i % self.eval_freq == 0:
                self.trainer.eval()
        trained_networks = self.trainer.policies
        save_model(trained_networks, PATH)



        