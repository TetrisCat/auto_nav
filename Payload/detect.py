import cv2
import numpy as np

cv2.ocl.setUseOpenCL(False)
#where is the image stored
image=cv2.imread('target.jpg')

#range of BGR values. cuz apparently opencv represents images as Np arrays in reverse order?

#cv2.imshow('image',image)
lower=np.array([17,14,100], dtype = "uint8")
upper=np.array([50,56,200],dtype = "uint8")
print(lower)
print(upper)
mask=cv2.inRange(image, lower, upper) # binary mask of white n black pixel
result=cv2.bitwise_and(image, image, mask=mask)
#cv2.imshow('images',np.hstack([image,result]))
cv2.waitKey(0)
print(mask)
#cv2.imshow('result',result)
result=np.array(result)  
#cv2.imshow('result2',result)
contours=cv2.findContours(result,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
#print(contours)
x,y,w,h=cv2.boundingRect(np.float32([[0,0],[0,2],[2,0],[2,2]])) #returns rectangle object with xy cor of top left corner n width n height parameter
print(x)
print(w)
print(h)
if w==3 and h==3:
    al_Cor=(x+0.5*w,y+0.5*h)
    get_loc= 3
    get_dir= 4 
print(al_Cor)
print(get_loc)
print(get_dir)

	




