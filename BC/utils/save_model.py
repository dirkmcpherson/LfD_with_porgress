import torch
import os


def save_model(model_list, PATH):
    path_name = os.path.join(PATH, "trained_models.pth")
    model_dict = {"model_{}".format(i):model for i,model in enumerate(model_list)}
    torch.save(model_dict, path_name)

def load_model(PATH):
    path_name = os.path.join(PATH, "trained_models.pth")
    model_dict = torch.load(path_name)
    model_list = [model_dict["model_{}".format(i)] for i in range(len(model_dict))]
    return model_list