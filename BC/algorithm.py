from agent import BCND_Trainer
from utils.bc_utils import get_all_bag_files,whole_bag_to_messages, read_one_trajectory_to_each_buffer
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
        msgs = whole_bag_to_messages(ls)
        read_one_trajectory_to_each_buffer(num_networks,self.trainer.buffers, msgs)

    def train(self):
        for i in range(self.iteration_num):
            self.trainer.run_one_iterarion()
            if i % self.eval_freq == 0:
                self.trainer.eval()
        trained_networks = self.trainer.policies
        save_model(trained_networks, PATH)



        