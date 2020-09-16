#!/usr/bin/env python

import tensorflow as tf
import keras
import rospy 
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import sys
import math
import copy
from project_practice_1.srv import project_practice1
from project_practice_1.srv import project_practice1Request
from project_practice_1.srv import project_practice1Response

bridge=CvBridge()
result=[]
centers=[]

def filter_color(rgb_image,lower_bound_color,upper_bound_color):
    hsv_image=cv2.cvtColor(rgb_image,cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image",hsv_image)
    mask=cv2.inRange(hsv_image,lower_bound_color,upper_bound_color)
    return mask
def get_contours(binary_image):
    _,contours,hierachy=cv2.findContours(binary_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    return contours,hierachy

def largecontour(rgb_image,contours,hierarchy):   
    #print(hierarchy)
    num=0
    global centers
    centers=[]     
    for index,c in enumerate(contours):
        area=cv2.contourArea(c)
        x,y,w,h=cv2.boundingRect(c)
        if area>300:
            cv2.drawContours(rgb_image,[c],-1,(0,255,255),1)
            cv2.rectangle(rgb_image,(x,y),(x+w,y+h),(0,0,255),1)
            print index
            if hierarchy[0][index][3]==-1:
                pix_x=x+w/2
                pix_y=y+h/2
                center=[pix_x,pix_y]
                centers.append(center)
                num+=1
                #depth info
    print("number of contours: {}".format(len(contours)))
    cv2.imshow("RGB Image Contours",rgb_image)
    #print(new_img_array.shape)
    return centers,num

def smallcontour(rgb_image,contours,hierarchy,yanglist):
    num_image=[]
    x_index=[]
    #print(hierarchy)
    for index,c in enumerate(contours):
        area=cv2.contourArea(c)
        x,y,w,h=cv2.boundingRect(c)
        if area>300:
            for yang in yanglist:
                if hierarchy[0][index][3]==yang:
                    print(yang)
                    temp=rgb_image[y-10:y+h+10,x-10:x+w+10]
                    #cv2.imshow('temp',temp)
                    temp=cv2.resize(temp,(28,28),interpolation=cv2.INTER_AREA)
                    num_image.append(temp)
                    x_index.append(x)
                    #print(type(num_image))
        #num_image=num_image[1:,:,:]
    num_image=np.array(num_image)
    #print(num_image.shape)
    return num_image,x_index
    
def yangyang(hierarchy):
    yanglist=[]
    for i,h in enumerate(hierarchy[0]):
        if h[3]==-1:
            #print(i)
            if i==len(hierarchy[0])-1:
                continue
            else:
                if hierarchy[0][i+1][3]==-1:
                    continue
                else:
                    for j in range(10):
                        if j==0:
                            continue
                        if i+j>=len(hierarchy[0]):
                            break
                        else: 
                            if j==1:
                                temp=hierarchy[0][i+j][3]
                                yanglist.append(temp)
                            else:
                                if hierarchy[0][i+j][3]==-1:
                                    break
                                else:
                                    if hierarchy[0][i+j][3]<temp:
                                        yanglist.append(hierarchy[0][i+j][3])
                                    else: 
                                        continue
    return yanglist                  

def handle_hierarchy(hierarchy,contours,yanglist):
    hList=[]
    for index,c in enumerate(contours):
        area=cv2.contourArea(c)
        if area>300:
            hList.append(hierarchy[0][index][3])
    #print(hList)
    fine_list=handle_hlist(hList,yanglist)
    return fine_list

def handle_hlist(hlist,yanglist):
    index=[]
    for i,h in enumerate(hlist):
        if h==-1:
            index.append(i)
    
    #print('index',index)
    fine_list=[]
    count=0
    c2=0
    for j,k in enumerate(index):
        if j==0 and len(index)!=1:
            new_list=hlist[:index[j]]
        if j==0 and len(index)==1:
            new_list=hlist
        else:  
            new_list=hlist[index[j-1]:index[j]]
        #print('new_list',new_list)
        
        for i in new_list:
            for y in yanglist:
                if i==y:
                    count+=1
        print count
        if (j==0 and len(index)!=1 and k!=0):
            fine_list.append(count)
        elif j==0 and len(index)==1:
            fine_list.append(count)
        elif j!=0:
            fine_list.append(count)    
            if j==len(index)-1:
                new_list2=hlist[index[j]:]
                #print(new_list2)
                for i in new_list2:
                    for y in yanglist:
                        if i==y:
                            c2+=1
                print(c2)
                fine_list.append(c2)
    print(fine_list)
    return fine_list

def sunseo(x_index,fine_list):
    label=[]
    print(x_index)
    for i,f in enumerate(fine_list):
        temp_list=[0]*f
        if i==0:
            temp_x=x_index[:f]
        else:
            temp_x=x_index[fine_list[i-1]:f+fine_list[i-1]]
        print(temp_x)
        sort_x=copy.deepcopy(temp_x)
        sort_x.sort()
        sort_x.reverse()
        #print(sort_x)
        #print(type(sort_x))
        for a,x in enumerate(temp_x):
            for s,s_x in enumerate(sort_x):
                if s_x==x:
                    temp_list[a]=s
                    print(s)
        label.append(temp_list)
    print('label',label)
    return label

def toBinary(img):
    gray_img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    _,binary_img=cv2.threshold(gray_img,107,255,cv2.THRESH_BINARY_INV)
    #cv2.imshow('binary',binary_img)
    return binary_img
def detection(fine_list,num_img,num,label):
    model=tf.keras.models.load_model('/home/jhmbabo/catkin_ws/src/project_practice_1/src/sibal.hdf5')
    #global model
    num_img=num_img/255.0
    prediction=model.predict(num_img)
    print(prediction)
    result=[0]*num
    if num==0:
        return result
    for n in range(num):
        #print(n)
        #print(fine_list[n])
        if n==0:
            temp_list=prediction[:fine_list[n]]
        else:
            temp_list=prediction[fine_list[n-1]:fine_list[n-1]+fine_list[n]]
        print(temp_list)
        for i,templist in enumerate(temp_list):
            if max(templist)<0.5:
                result[num-1-n]=0
            else:
                temp=np.argmax(templist)
                res=temp.item()
            #print(type(res))
                result[num-1-n]+=math.pow(10,label[n][i])*res
    print('result',result)
    return result

def callback(ros_image):
    print 'got an image'
    global bridge
    try:
        cv_image=bridge.imgmsg_to_cv2(ros_image,"bgr8")
    except CvBridgeError as e:
        print(e)
    Lower=(40,50,90)
    Upper=(70,255,255)
    mask=filter_color(cv_image,Lower,Upper)
    cv2.imshow('mask',mask)
    countours,hierarchy=get_contours(mask)
    yanglist=yangyang(hierarchy)
    centers,num=largecontour(cv_image,countours,hierarchy)
    num_img,x_index=smallcontour(cv_image,countours,hierarchy,yanglist)
    fine_list=handle_hierarchy(hierarchy,countours,yanglist)
    label=sunseo(x_index,fine_list)
    new_num_image=np.zeros((len(num_img),28,28,1))
    #number=np.arange(1,num+1)
    for index,pix in enumerate(num_img):
        print(pix.shape)
        #cv2.imshow('pix',pix)
        binary=toBinary(pix)
        cv2.imwrite("/home/jhmbabo/catkin_ws/src/numpy_tutorial/src/img/num_image_"+str(7*10+index)+".jpg",binary)
        binary=np.reshape(binary,(28,28,1))
        new_num_image[index]=binary
    global result
    result=detection(fine_list,new_num_image,num,label)
    
    cv2.waitKey(10)
    global image_sub
    image_sub.unregister()

def callback_1(ros_image):
    result=[]

image_sub=rospy.Subscriber("/usb_cam/image_raw",Image,callback_1)

def handler(req):
    global image_sub
    image_sub=rospy.Subscriber("/usb_cam/image_raw",Image,callback)
    input_num=req.input
    input_num=float(input_num)
    for index,res in enumerate(result):
       if res==input_num:
            detect=True
            if centers[index][0]<320:
                dire=1
            else:
                dire=2
            return project_practice1Response(detect,dire)
    detect=False
    dire=0
    
    return project_practice1Response(detect,dire)


def main(args):
    rospy.init_node('preprocessing',anonymous=True)
    #global image_sub
    #image_sub=rospy.Subscriber("/usb_cam/image_raw",Image,callback)
    
    server=rospy.Service('num_detection',project_practice1,handler,buff_size=10)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)