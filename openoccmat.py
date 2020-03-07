import re
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

f = open('occ_mat.txt')
line = f.readlines()[11]
f.close()
occdata = np.array(re.findall('\d+',line), dtype=int)

occ_mat = occdata.reshape(384,384,order='F')
# occ_mat = np.array([[0,0,0,0,0,0,0,0,0],
#               [0,0,0,0,0,0,1,1,0],
#               [0,2,2,2,2,2,1,2,0],
#               [0,2,1,1,1,1,1,2,0],
#               [0,2,2,2,2,2,2,2,0],
#               [0,0,0,0,0,0,0,0,0]])

print(occ_mat)

img = Image.fromarray(occ_mat.astype(np.uint8))
rotated = img.rotate(180)


x= {coord:value for coord,value in np.ndenumerate(occ_mat)}
for k,v in x.items():
    if v == 2:
        i = k[0]
        j = k[1]
        counter = 0
        checkers = [-1,1]
        for checker in checkers:
            if x[(i+checker,j)] == 2:
                counter +=1
            if x[(i,j+checker)] == 2:
                counter +=1
        if counter <2:
            print(k)


plt.imshow(img,cmap='gray')
plt.draw_all()
plt.show()