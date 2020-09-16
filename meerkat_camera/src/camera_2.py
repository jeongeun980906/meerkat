#!/usr/bin/env python
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
#from tf.transformations import euler_from_quaternion, quaternion_from_euler,euler_matrix,quaternion_matrix
from meerkat_camera.msg import ccam_kf,ccam_rl
import rospy
import random
import numpy as np
import cv2
from math import cos,sin,pi,sqrt,atan2

#from kinematics import forward

class kinect_rgb_img_get():
    def __init__(self):
        ############ Camera Parameters ############ 
        self.ppx=314.25146484375
        self.ppy =237.777557373047
        self.fu=612.405334472656
        self.fv=610.648681640625
        #self.ppx = 638.384765625
        #self.ppy = 367.5414123535156
        #self.fu = 613.1182250976562
        #self.fv = 613.0377807617188
        self.camera_parameters = (self.ppx, self.ppy, self.fu, self.fv)
        ###########################################

        self.bridge = CvBridge()
        self.image = Image()
        self.flag, self.flag_depth = False, False
        self.count = 0
        self.pub_rl_pose = rospy.Publisher('position_rl', ccam_rl, queue_size=5)
        self.pub_kf_pose = rospy.Publisher('position_kf', ccam_kf, queue_size=5)
        #rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_call_back)
        #rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_call_back)
        self.depth=np.zeros((480,640))
        self.box_list_state=[[0,0,0,0],[0,0,0,0]]

        self.tansformaer=trans()


    def pixel_to_tool(self, cam_param, pos, pos_z):
        ppx, ppy, fu, fv = cam_param
        theta=0.25*pi
        x_pix = pos[0]
        y_pix = pos[1]
        #z_rgb = pos_z ##mm
        #print(x_pix,y_pix)
        x_rgb = pos_z*(x_pix-ppx)/fu # mm
        y_rgb = pos_z*(y_pix-ppy)/fv # mm
        #print(x_rgb,y_rgb)
        z_rgb=sqrt(pos_z**2-x_rgb**2-y_rgb**2)
        ########################
        #y = -y_rgb*sin(theta)+z_rgb*cos(theta)
        #x =-x_rgb

        x = -x_rgb*sin(theta)+z_rgb*cos(theta)
        y =-y_rgb
        ########################
        #print(x_rgb,y_rgb,z_rgb)
       # print('x,y',x,y)
        # z = z_rgb - 184.0
        return x, y



    def rgb_call_back(self, ros_img):

        self.cv2_rgb_image = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")
        #print('callback1')
        self.flag = True


    def call_back(self):
        data1 = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=1)
        data2=rospy.wait_for_message('/camera/depth/image_rect_raw', Image, timeout=1)
        self.cv2_rgb_image = self.bridge.imgmsg_to_cv2(data1, "bgr8")
        self.cv2_depth_image = self.bridge.imgmsg_to_cv2(data2, "passthrough")
        #print('callback2')
        #self.flag_depth = True
        if True:#self. flag==True and self.flag_depth==True:
            msg1=ccam_rl()
            msg2=ccam_kf()
            x1,y1,x2,y2=self.get_pose_class_state()
            resx1,resy1=self.tansformaer.transform_rl(x1,y1,x2,y2)
            resx2,resy2,resyaw=self.tansformaer.transform_kf(x1,y1,x2,y2)
            msg1.x=resx1
            msg1.y=resy1
            msg2.x=resx2
            msg2.y=resy2
            msg2.yaw=resyaw
            self.pub_rl_pose.publish(msg1)
            self.pub_kf_pose.publish(msg2)

            #print(self.position_class_state)

        # print("self.cv2_depth_image.shape :", self.cv2_depth_image.shape) # ('self.cv2_depth_image.shape :', (720, 1280))



    def depth_interpolation(self):
        depth = self.cv2_depth_image
        depth = depth.astype(np.uint8)
        mask = np.zeros(depth.shape)
        for i in range(depth.shape[0]):
            for j in range(depth.shape[1]):
                if depth[i,j] == 0 :
                    mask[i,j] = 255
        #print(depth.shape)
        mask = mask.astype(np.uint8)
        # depth_tmp = depth_tmp.astype(np.uint8)
        dst = cv2.inpaint(depth ,mask, 3, flags=cv2.INPAINT_TELEA)
        self.depth = dst



    def get_pose_class_state(self):
        #self.depth_interpolation()
        self.position_class_state = []
        #print(self.depth)

        mask1=self.filter_color(self.cv2_rgb_image,0)
        contours1=self.get_contours(mask1)
        self.process_contours(self.cv2_rgb_image,mask1,contours1,0)
        mask2=self.filter_color(self.cv2_rgb_image,1)
        contours2=self.get_contours(mask2)
        self.process_contours(self.cv2_rgb_image,mask2,contours2,1)
        cv2.imshow("RGB Image",mask1)
        cv2.imshow("RGB Image3",mask2)
        cv2.imshow("RGB Image2",self.cv2_rgb_image)
        cv2.waitKey(3)
        #x1,x2,y1,y2=0,0,0,0
        for i in range(2):    
            (x,y,w,h)=self.box_list_state[i]
            x_pix = int(x+w/2)
            y_pix = int(y+h/2)

            # box_data = self.bounding_boxes_main[i]
            # x_pix = int((box_data[0].xmin + box_data[0].xmax)/2)
            # y_pix = int((box_data[0].ymin + box_data[0].ymax)/2)

            obj_pix = (x_pix, y_pix)
            #z_cam = self.depth[y_pix, x_pix]
            z_cam=self.cv2_depth_image[y_pix, x_pix]
           # print(z_cam,i)
            # z_cam = self.depth[y_pix, x_pix] * 0.001

            position_x, position_y = self.pixel_to_tool(self.camera_parameters, obj_pix, z_cam)
            # position_z = z_cam - 0.17
            if i==0:
                x2=position_x
                y2=position_y
            if i==1:
                x1=position_x
                y1=position_y
            # print('POS_state / Class : ', -position_y, -position_x, position_z, " / ", self.class_list_state[i])

            #self.obj_position_class_state.append((-position_y, -position_x, position_z, self.class_list_state[i]))
        return x1,y1,x2,y2


    def filter_color(self,rgb_image,i):
        if i==0:
            LowerBound=(20,100,100)
            UpperBound=(50,255,255)
        else:
            LowerBound=(30,70,100)
            UpperBound=(80,230,250)
                
        hsv_image=cv2.cvtColor(rgb_image,cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(hsv_image,LowerBound,UpperBound)
        return mask

    def get_contours(self,binary_image):
        _,contours,hierachy=cv2.findContours(binary_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        return contours
    
    
    def process_contours(self,rgb_image,binary_image,contours,i):
        black_image=np.zeros([rgb_image.shape[0],rgb_image.shape[1],3],'uint8')
        area_max=0
        for c in contours:
            area=cv2.contourArea(c)
            x,y,w,h=cv2.boundingRect(c)
            cv2.drawContours(rgb_image,[c],-1,(0,255,255),1)
            #cv2.rectangle(rgb_image,(x,y),(x+w,y+h),(0,0,255),1)
            if area>10 and area_max<area: #need to set area
                area_max=area
                cv2.drawContours(rgb_image,[c],-1,(0,255,255),1)
                cv2.rectangle(rgb_image,(x,y),(x+w,y+h),(0,0,255),1)
                self.box_list_state[i]=[x,y,w,h]

            #print("Area:{}".format(area))
class trans():
    def __init__(self):
        self.theta=0.666*pi#1.66667*pi
        self.xc=3800
        self.yc=1878#2678
        self.cx0=-sin(self.theta)*self.yc-self.xc*cos(self.theta) #object seen from camera frame
        self.cy0=-cos(self.theta)*self.yc+self.xc*sin(self.theta)

    def transform_rl(self,x1,y1,x2,y2):
        yaw=atan2(y1-y2,x1-x2)
        x=(x1+x2)/2
        y=(y1+y2)/2
        x_t=cos(yaw)*(-x+self.cx0)-sin(yaw)*(y-self.cy0) #object seen from body frame
        y_t=sin(yaw)*(x-self.cx0)-cos(yaw)*(y-self.cy0)
        return x_t,y_t

    def transform_kf(self,x1,y1,x2,y2):
        #print('w',x1,x2,y1,y2)
        yaw=atan2(x1-x2,y1-y2)
        bxc=(x1+x2)/2         #body from cam frame
        byc=(y1+y2)/2
        #print(bxc,byc,yaw,x1-x2,y1-y2)
        #bxo=(cos(self.theta)*bxc-sin(self.theta)*byc)*1.1+self.xc #body from object frame
        #if yaw<-1.5:
        byo=-sin(self.theta)*bxc-cos(self.theta)*byc+self.yc
        bxo=(cos(self.theta)*bxc-sin(self.theta)*byc)+self.xc-1.5*(byo+0.5)
        byaw=self.theta+yaw+0.5*pi
        return bxo/1000,byo/1000,byaw


def main():
    rospy.init_node("camer_node")

    image_processor=kinect_rgb_img_get()
    for _ in range(10000):
        image_processor.call_back()
        #obj_position=kinect_rgb_img_get.get_pose_class_state()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "shutting down"
    #cv2.destroyAllWindows()

if __name__=='__main__':
    main()