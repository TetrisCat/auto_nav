import math
import numpy as np 
import time

signal = [[1,0,2,0,2,1,2,3],[1,0,4,2,1,3,2,1],[2,1,2,4,0,2,1,3],[1,3,2,1,4,2,0,1]]


def convolution(signal):
    new_signal = []
    final_sig = []
    for row in signal:
        signal_row = [num if num  <2 else 200 for num in row]
        new_signal.append(signal_row)
        final_sig.append([])

    signal = new_signal
    print(signal)
    for i,j in np.ndenumerate(signal):
        if i[0] == 0:
            pass
        elif i[0] == len(signal) - 1 :
            pass
        elif i[1] == 0:
            pass
        elif i[1] == len(signal[0]) -1:
            pass
        else:
            val = signal[i[0]-1][i[1]-1] + signal[i[0]][i[1]-1] +signal[i[0]+1][i[1]-1] +\
                signal[i[0]-1][i[1]] + signal[i[0]][i[1]] + signal[i[0]+1][i[1]] + \
                    signal[i[0]-1][i[1]+1] + signal[i[0]][i[1]+1] + signal[i[0]+1][i[1]+1]
            final_sig[i[0]].append(val)
    return final_sig



print(convolution(signal))
