#%%
import pickle
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline, BSpline
from scipy.ndimage.filters import gaussian_filter1d
from scipy.interpolate import interp1d
def load_pickle(filename):
    with open(filename, 'rb') as handle:
        b = pickle.load(handle)
    return(b)




def smooth(path):
    x=list()
    y=list()
    for i in range(len(path)-1,-1,-1):
        y.append(path[i][0])
        x.append(path[i][1])
    ysmoothed = gaussian_filter1d(y, sigma=2)
    new_path=[(y,x) for y,x in zip(ysmoothed,x)]
    return(new_path)

def detect_patern(path):
    not_same_dir=False
    elt_to_remove=list()
    #Detect if 3 autour
    for i in range(len(path)-2,0,-1):
        prev=path[i+1]
        curr=path[i]
        next=path[i-1]
        if(prev[1]!=curr[1] and curr[0]!=next[0]):
            elt_to_remove.append(curr)
    p=path.copy()        
    for elt in elt_to_remove:
        p.remove(elt)
    return(p)

def extract_xy(path):
    x=list()
    y=list()
    for i in range(len(path)-1,-1,-1):
        y.append(path[i][0])
        x.append(path[i][1])
    return(x,y)

if __name__=='__main__':
    filename="path.pickle"
    path=load_pickle(filename)
    new_path=detect_patern(path)
    print(len(new_path)==len(path))
    
    fig, axs = plt.subplots(2)
    x,y=extract_xy(path)
    axs[0].plot(x,y,'-ok')
    x1,y1=extract_xy(new_path)
    axs[1].plot(x1,y1,'-ob')
    plt.show()

    """
    fig, axs = plt.subplots(3)
    #plt.plot(x,y,"-ok")
    x=list()
    y=list()
    for i in range(len(path)-1,-1,-1):
        y.append(path[i][0])
        x.append(path[i][1])
    ysmoothed = gaussian_filter1d(y, sigma=2)

    axs[0].plot(x, ysmoothed,'-ok')
    axs[1].plot(x,y,'-ob')

    print(len(x))
    print(len(ysmoothed))
    plt.show()
    """

# %%
