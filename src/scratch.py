import pickle

filepath = '../calibration/transformation_table.pkl'
with open(filepath,'rb') as f:
    print(pickle.load(f))