import cv2
import numpy as np

#where is the image stored
image=cv2.imread('target.jpg')

#range of BGR values. cuz apparently opencv represents images as Np arrays in reverse order?
bound=[([17,14,100],[50,56,200])]

for (lower,upper) in bound:
    lower=np.array(lower, dtype = "uint8")
    upper=np.array(upper,dtype = "uint8")
    mask=cv2.inRange(image, lower, upper) # binary mask of white n black pixels
    print(mask)
    contours=cv2.findContours(mask.astype('int'),mode="CV_RETR_CCOMP",method="CV_CHAIN_APPROX_SIMPLE")
    target=cv2.boundingRect(contours) #returns rectangle object with xy cor of top left corner n width n height parameter
    if target[2,3]==[(["pixel range"],["pixel range"])]:
        al_Cor=(target[2]-target[0],target[3]-target[1])
        get_loc= 3
        get_dir= 4 

	




