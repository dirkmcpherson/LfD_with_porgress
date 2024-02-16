from algorithm import BCND_algorithm


iter_num = 1
obs_dim =9
action_dim = 3
batch_size = 256
max_buffer_size = 10000
training_horizon = 2000
learning_rate = 1e-4
num_networks = 1
network_config = dict(
    hidden_layer_dimension = 100,
    hidden_layer_numbers = 2
)
eval_freq = 1
algo = BCND_algorithm(
    iter_num,
    obs_dim,
    action_dim,
    max_buffer_size,
    batch_size,
    training_horizon,
    learning_rate, 
    num_networks, 
    network_config,
    eval_freq
)
algo.train()