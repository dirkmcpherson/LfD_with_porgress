from algorithm import BCND_algorithm


iter_num = 20
obs_dim =9
action_dim = 3
batch_size = 200
max_buffer_size = 100000
training_horizon = 5000
learning_rate = 1e-4
num_networks = 3
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