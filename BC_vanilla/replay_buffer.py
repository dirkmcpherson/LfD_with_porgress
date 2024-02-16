import numpy as np
from utils.bc_utils import read_bag

class SimpleReplayBuffer():
    def __init__(self, obs_dim, action_dim, max_buffer_size:int) -> None:
        self.max_buffer_size = max_buffer_size
        if isinstance(obs_dim, int):
            obs_dim = [obs_dim]
            # obs_dim = list(obs_dim)
        if isinstance(action_dim, int):
            action_dim = [action_dim]        
        self._observartion_dim = obs_dim
        self._action_dim = action_dim

        self.observations = np.zeros((max_buffer_size,*self._observartion_dim),dtype=np.float64)
        self.actions = np.zeros((max_buffer_size, *self._action_dim),dtype=np.float64)
        self._top = 0
        self._size = 0
    def load_buffer_from_bag(self, bag_path:str):
        bag_messages = read_bag(bag_path)
        for i in range(len(bag_messages) - 1):
            self.observations[i][:] = bag_messages[i].position[:self._observartion_dim]
            self.actions[i][:] = bag_messages[i+1].position[:self._observartion_dim] - bag_messages[i].position[:self._observartion_dim]
        # last state doesn't have action
        self.observations[len(bag_messages)-1][:] = bag_messages[len(bag_messages)-1].position[:self._observartion_dim]
        self.actions[len(bag_messages)-1][:] = 0

    def add_sample(self, obs, action):
        self.observations[self._top] = obs
        self.actions[self._top] = action
        self._advance()

    def _advance(self):
        self._top = (self._top+1) % self.max_buffer_size
        if self._size < self.max_buffer_size:
            self._size += 1

    def add_rollout(self, observation_list:np.ndarray, action_list:np.ndarray):
        assert self.observations[0].size == observation_list[0].size, "different size for observations!\n"
        assert self.actions[0].size == observation_list[0].size, "different size for actions!\n"
        assert len(observation_list) == len(action_list), "observations and actions have differnet length!"
        for i,(obs,action) in enumerate(zip(observation_list,action_list)):
            self.add_sample(obs, action)
            

    def random_sample(self, batch_size):
        # print(self._size)
        randnum = np.random.randint(0, self._size, batch_size)
        batch = dict(
            observations = self.observations[randnum], 
            actions = self.actions[randnum]
        )
        return batch
    @property
    def size(self):
        return self._size
    


