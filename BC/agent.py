import numpy as np
from replay_buffer import SimpleReplayBuffer
from network import BCND_network
import torch
import utils.bc_utils
import torch.nn.functional as functional

DEVICE = utils.bc_utils.device()

class BCND_Trainer():
    def __init__(
            self,
            obs_dim,
            action_dim,
            max_buffer_size,
            batch_size,
            training_horizon,
            learning_rate:float,
            num_networks:int,
            network_config:dict
        ) -> None:
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.hidden_dim = network_config["hidden_layer_dimension"]
        self.hidden_layers = network_config["hidden_layer_numbers"] 
        self.batch_size = batch_size  
        self.num_policies = num_networks 
        self.lr = learning_rate   
        self.training_horizon = training_horizon
        self.max_buffer_size = max_buffer_size
        # flatten to a number if in_dim or action_dim is high-dimensional 
        if not isinstance(obs_dim, int):
            self.obs_dim = np.prod(obs_dim)
        if not isinstance(action_dim, int):
            self.action_dim = np.prod(action_dim)

        self.buffers = []
        for i in range(self.num_policies):
            self.buffers.append(SimpleReplayBuffer(
                self.obs_dim,
                self.action_dim, 
                self.max_buffer_size))



        self.optimizer = torch.optim.Adam
        # self.policy = BCND_network(self.obs_dim, self.action_dim, self.hidden_dim, self.hidden_layers)
        # self.policy.to(DEVICE)

        # a list of policies
        self.policies = []
        # old policies for rewards
        self.old_policies = []
        # initialize old policies as random networks
        for _ in range(num_networks):
            network_k = BCND_network(self.obs_dim, self.action_dim, self.hidden_dim, self.hidden_layers).to(DEVICE)
            self.old_policies.append(network_k)



    # BCND reward from old policies and sampled experiences
    def reward(self, observations, actions:torch):
        rewards = []
        actions_tensor = torch.tensor(actions).to(DEVICE)
        for i in range(self.num_policies):
            reward_k_mean, reward_k_std = self.old_policies[i](observations)
            reward = torch.distributions.Normal(reward_k_mean, reward_k_std)
            assert actions_tensor.size() == reward_k_mean.size()
            log_prob_old:torch.Tensor = reward.log_prob(actions_tensor)
            prob_old = log_prob_old.exp()
            rewards.append(prob_old)
        reward_tensor =torch.stack(rewards)
        reward_mean = reward_tensor.mean(dim=0,keepdim=False)
        # print("reward_mean: {}\n".format(reward_mean))
        return reward_mean.detach()
    

        
            



    def run_batch(self, policy:BCND_network, buffer:SimpleReplayBuffer):
        # TODO: modify to K replay buffers
        batch = buffer.random_sample(self.batch_size)
        observations:np.ndarray = batch["observations"]
        actions = batch["actions"]
        assert np.size(observations, axis=1) == self.obs_dim, "ERROR: Observations from replay buffer have wrong dimension!\n"
        assert np.size(actions, axis = 1) == self.action_dim, "ERROR: Actions from replay buffer have wrong dimension!\n"
        observations = torch.from_numpy(observations).float().to(DEVICE)
        actions_tensor = torch.tensor(actions).to(DEVICE)
        # tst_obs = torch.tensor(np.ones((100,6),dtype=float)*100).float().to(DEVICE)
        predicted_action_mean, predicted_action_std = policy(observations)
        # print("means:{}\n".format(predicted_action_mean))
        # print("stds:{}\n".format(predicted_action_std)) 
        
        policy_dist = torch.distributions.Normal(predicted_action_mean,predicted_action_std)
        # print(list(actions_tensor.size()),'\n')
        # print(list(predicted_action_mean.size()),
            # '\n')
        assert actions_tensor.size() == predicted_action_mean.size()
        log_prob_action_n = policy_dist.log_prob(actions_tensor)
        rewards = self.reward(observations, actions)
        # print("rewards:{}\n".format(rewards.mean()))
        # print("prob:", log_prob_action_n)
        # print("prob_size:",log_prob_action_n.size())
        # print("reward_size:",rewards.size())
        # print("log_prob_action_n:{}\n".format(log_prob_action_n.exp().mean()))
        loss = -(log_prob_action_n * rewards)
        loss = loss.mean()        
        # print("loss:",loss)

        optimizer = self.optimizer(policy.parameters(), self.lr)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

    def update_old_policies(self):
        self.old_policies = self.policies


    
    def create_policy(self):
        # TODO: use other function to make sure policy list num is correct
        policy = BCND_network(self.obs_dim, self.action_dim, self.hidden_dim, self.hidden_layers).to(DEVICE)
        self.policies.append(policy)
        return policy

        
    def run_one_iterarion(self):
        self.policies = []
        for k in range(self.num_policies):
            # print("network:{}\n".format(k))
            policy = self.create_policy()
            buffer:SimpleReplayBuffer = self.buffers[k]
            for l in range(self.training_horizon):
                self.run_batch(policy, buffer)
        self.update_old_policies()
        

    # evaluating policy performance to check whether training is going
    def eval(self):
        # for policy in self.old_policies:
        #     policy.eval()
        buffer:SimpleReplayBuffer = self.buffers[0]
        # observations_whole = buffer.observations
        random_batch = buffer.random_sample(self.batch_size)
        observations_whole = random_batch["observations"]
        actions_whole = random_batch["actions"]
        random_actions = buffer.random_sample(self.batch_size)["actions"]
        randoom_actions = torch.tensor(random_actions).to(DEVICE)
        # shift obs to get next obs, leave the last one as 0
        # next_obs = np.concatenate((observations_whole[1:],np.zeros((1,self.obs_dim))),axis=0)
        # next_obs_tensor = torch.tensor(next_obs).to(DEVICE)
        # actions_whole = buffer.actions
        # actions_whole = buffer.random_sample(self.batch_size)["actions"]
        observations_whole_tensor = torch.from_numpy(observations_whole).float().to(DEVICE)
        actions_whole_tensor = torch.tensor(actions_whole).to(DEVICE)
        reward_predictions = self.reward(observations_whole_tensor, actions_whole_tensor)
        random_action_reward = self.reward(observations_whole_tensor, random_actions)
        # print("reward_predictions:{}\n".format(reward_predictions))
        # print("next_obs_tensor:{}\n".format(next_obs_tensor))
        # loss:torch.Tensor = creterion(reward_predictions,next_obs_tensor)
        # # print("loss:{}\n".format(loss))
        # loss_avg = loss/next_obs_tensor.size(0)
        # loss_avg = loss_avg.item()
        # print("average MSE loss over one trajectory:{}\n".format(loss_avg))
        reward = reward_predictions.mean().detach().cpu().numpy()
        random_acrtion_reward = random_action_reward.mean().detach().cpu().numpy()
        print("reward:{}\n".format(reward))
        print("reward for random actions:{}\n".format(random_acrtion_reward))



            
    




