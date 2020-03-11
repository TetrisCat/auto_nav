import math
import numpy as np 
import time
from scipy import signal
res = 0.05
origin = (-10.0,-10.0)
sig = [[1,0,2,0,2,1,2,3],
    [1,0,4,2,1,3,2,1],
    [2,1,2,4,0,2,1,3],
    [1,3,2,1,4,2,0,1]]

adjusted = []

cur_pose = (0,0)

for row in sig:
    x = [num if num == 1 else 0 if num >=2 else 10 for num in row]
    adjusted.append(x)

kernel = np.zeros((3,3)) + 1
print(kernel)

output = signal.convolve2d(adjusted, kernel, boundary='fill',mode='same')
def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def get_closest(original,adjusted,curpos,res,origin):
    lst = []
    for coord,val in np.ndenumerate(adjusted):
        row = coord[0]
        col = coord[1]
        if val % 10 != 0 and original[row][col] != 2:
            lst.append((col*res + origin[0],row*res + origin[1]))
    
    distance_lst = []
    
    for crd in lst:
        distance_lst.append(distance(crd,curpos))
    idx = np.argmin(distance_lst)
    return lst[idx]


print(get_closest(sig,output,cur_pose,res,origin))
