#!/usr/bin/env python
PKG = 'numpy_tutorial'
import roslib; roslib.load_manifest(PKG)

import rospy 
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import numpy as np
import cv2

def filter_color(rgb_image,lower_bound_color,upper_bound_color):
    hsv_image=cv2.cvtColor(rgb_image,cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image",hsv_image)
    mask=cv2.inRange(hsv_image,lower_bound_color,upper_bound_color)
    return mask
def get_contours(binary_image):
    _,contours,hierachy=cv2.findContours(binary_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours,hierachy

def largecontour(rgb_image,contours,hierarchy):   
    print(hierarchy)
    num=0
    new_img=[]     
    for index,c in enumerate(contours):
        area=cv2.contourArea(c)
        x,y,w,h=cv2.boundingRect(c)

        if area>100:
            cv2.drawContours(rgb_image,[c],-1,(0,255,255),1)
            cv2.rectangle(rgb_image,(x,y),(x+w,y+h),(0,0,255),1)
            print index
            if hierarchy[0][index][3]==-1:
                print 'here'
                temp=rgb_image[y:y+h,x:x+w]
                temp=cv2.resize(temp,(160,160),interpolation=cv2.INTER_CUBIC)
                #cv2.imshow('temp',temp)
                new_img.append(temp)
                num+=1
    print("number of contours: {}".format(len(contours)))
    cv2.imshow("RGB Image Contours",rgb_image)
    new_img_array=np.asarray(new_img)
    print(new_img_array.shape)
    return new_img_array,num

def smallcontour(rgb_image,contours,hierarchy):
    num_image=[]
    print(hierarchy)
    for index,c in enumerate(contours):
        area=cv2.contourArea(c)
        x,y,w,h=cv2.boundingRect(c)
        if area>100:
            #cv2.drawContours(new_image,[c],-1,(0,255,255),1)
            #cv2.rectangle(new_image,(x,y),(x+w,y+h),(0,0,255),1)
            if hierarchy[0][index][3]==0:
                temp=rgb_image[y:y+h,x:x+w]
                #cv2.imshow('temp',temp)
                temp=cv2.resize(temp,(28,28),interpolation=cv2.INTER_CUBIC)
                num_image.append(temp)
                print(type(num_image))
    #num_image=num_image[1:,:,:]
    num_image=np.array(num_image)
    #print(num_image.shape)
    return num_image

def handle_hierarchy(hierarchy,contours):
    hList=[]
    for index,c in enumerate(contours):
        area=cv2.contourArea(c)
        if area>100:
            hList.append(hierarchy[0][index][3])
    fine_list=handle_hlist(hList)

def handle_hlist(hlist):
    index=[]
    for i,h in enumerate(hlist):
        if h==-1:
            index.append(i)
    
    print(index)
    fine_list=[]
    for j,k in enumerate(index):
        if j==0:
            new_list=hlist[:index[j]]
        else:  
            new_list=hlist[index[j-1]:index[j]]
        print(new_list)
        count=0
        for i in new_list:
            if i==0:
                count+=1
        print count
        if (j==0 and k!=0):
            fine_list.append(count)
        if j==len(index)-1:
            new_list2=hlist[index[j]:]
            print(new_list2)
            c2=0
            for i in new_list2:
                if i==0:
                    c2+=1
            fine_list.append(c2)
    print(fine_list)
    return fine_list

def toBinary(img):
    gray_img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    _,binary_img=cv2.threshold(gray_img,107,255,cv2.THRESH_BINARY_INV)
    #cv2.imshow('binary',binary_img)
    return binary_img

def main():
    image_name="/home/jhmbabo/catkin_ws/src/numpy_tutorial/src/img/image.jpg"
    img=cv2.imread(image_name)
    Lower=(0,0,0)
    Upper=(255,35,255)
    mask=filter_color(img,Lower,Upper)
    cv2.imshow('mask',mask)
    countours,hierarchy=get_contours(mask)
    #print(hierarchy)
    print type(img)
    new_img_array,num=largecontour(img,countours,hierarchy)
    #new_img=new_img_array[0]
    #print(num)
    #print(new_img.shape)
    #cv2.imwrite("/home/jhmbabo/catkin_ws/src/numpy_tutorial/src/img/new_img.jpg",new_img)
    #for i,num_img in enumerate(new_img_array):
        #num_img=smallcontour(img,countours,hierarchy)
    #for index,i in enumerate(num_img):
        #print i,index
        #print(i.shape)
        #cv2.imshow('i',i)
        #binary=toBinary(i)
        #cv2.imwrite("/home/jhmbabo/catkin_ws/src/numpy_tutorial/src/img/num_image_"+str(index)+".jpg",binary)
    for i,new_img in enumerate(new_img_array):
        cv2.imwrite("/home/jhmbabo/catkin_ws/src/numpy_tutorial/src/img/new_img_"+str(i)+".jpg",new_img)
   
    num_img=smallcontour(img,countours,hierarchy)
    fine_list=handle_hierarchy(hierarchy,countours)
    number=np.arange(1,num+1)
    for index,pix in enumerate(num_img):
        print(pix.shape)
        cv2.imshow('pix',pix)
        binary=toBinary(pix)
        cv2.imwrite("/home/jhmbabo/catkin_ws/src/numpy_tutorial/src/img/num_image_"+str(i*10+index)+".jpg",binary)

    #handle(hierarchy)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()