#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from scipy.linalg import svd
import time
import matplotlib.pyplot as plt
import math as m
# intialize the node
rospy.init_node("feature_extractor_node")
ini_image_obtained = False
prev_frame = None

# Parameters for ShiTomasi corner detection
feature_params = dict(maxCorners = 1200,
                      qualityLevel = 0.3,
                      minDistance = 7,
                      blockSize = 7)

# Parameters for lucas kanade optical flow
lk_params = dict(winSize = (15,15),
                maxLevel = 2,
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# camera calibration matrix
K_calib = np.array([[554.3827128226441, 0.0, 320.5],
                    [0.0, 554.3827128226441, 240.5],
                    [0.0, 0.0, 1.0]])
#create some random colors
color = np.random.randint(0, 255, (500, 3))

#define global state and Projection matrix P = [R,t] (3x4)
Projection_mat_homo = np.array([[1,0,0,0],
                                [0,1,0,0],
                                [0,0,1,0],
                                [0,0,0,1]])

statex = []
statey = []
state_theta = []

origx = []
origy = []

def update(x,y,theta,v,w):
    dt = 0.1
    x = x + v*m.cos(theta)*dt
    y = y + v*m.sin(theta)*dt
    theta = theta + w*dt
    return([x,y,theta])

def harris_corner(image):
    # convert the image to gray scale
    gray_img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

    #conversion to float is a prerequisite for the algorithm
    gray_img = np.float32(gray_img)

    # 3 is the size of the neighbourhood considerd, aperture parameter = 3
    # k = 0.04 used to calculate the window score (R)
    corners_img = cv2.cornerHarris(gray_img, 3, 3, 0.04)

    #mark the corners in green
    image[corners_img>0.001*corners_img.max()] = [0,255,0]

    return image

def shi_tomasi(image):
    gray_img =  cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # the parameters in the next line could be tuned if needed
    corner_img = cv2.goodFeaturesToTrack(gray_img,1200,0.01,10)

    for corners in corner_img:
        x, y = corners.ravel()
        cv2.circle(image,(x,y),3,[255,255,0],-1)

    return image

def findFundamentalMatrix2(pt1, pt2):
    """
    cv2 implementation that uses RANSAC method
    """
    F, mask = cv2.findFundamentalMat(
            pt1,
            pt2,
            cv2.FM_RANSAC
        )
    return F

def findFundamentalMatrix(go, gn): # short for good_old and good new
    # We know that fundamental matrix has to satisfy
    # x2.transpose() * F * x1 = 0
    # we have 8 such points and thus on expanding for anyone we will have  a linear equation
    # concatenating 8 such equations will give as A*f_vector = 0
    # whose solution would be the last vector in the V matrix of svd of A = UDV.tranpose()

    """
    NOTE that we are using cv2 optical flow features are good and thus don't need RANSAC algorithm
    if time permits we will find good features from RANSAC
    """
    A = np.array([[go[0,0]*gn[0,0] , go[0,1]*gn[0,0], 1*gn[0,0],
                  go[0,0]*gn[0,1] , go[0,1]*gn[0,0], 1*gn[0,0],
                  go[0,0] , go[0,1], 1]])

    for i in range(1,8):
        dummy = np.array([[go[i,0]*gn[i,0] , go[i,1]*gn[i,0], 1*gn[i,0],
                          go[i,0]*gn[i,1] , go[i,1]*gn[i,0], 1*gn[i,0],
                          go[i,0] , go[i,1], 1]])
        A = np.concatenate((A,dummy),axis=0)

    # find the svd of this A
    U, S,VT = svd(A)
    #create the sigma matrix
    Sigma = np.zeros((A.shape[0], A.shape[1])) #same size as A
    Sigma[:A.shape[0],:A.shape[0]] = np.diag(S)

    f_vector = VT.transpose()[:,-1]
    F = np.reshape(f_vector,(3,3))

    # before sending this F we need to perform SVD cleanup
    # find the svd of this A
    U, S,VT = svd(F)
    #create the sigma matrix
    Sigma = np.zeros((F.shape[0], F.shape[1])) #same size as A
    Sigma[:F.shape[0],:F.shape[0]] = np.diag(S)
    # need to make last row zero
    Sigma[2,0] = 0
    Sigma[2,1] = 0
    Sigma[2,2] = 0

    F = U.dot(Sigma.dot(VT))
    # print("F is: {}".format(F))
    return F

def extractRtFromF(F, pt_prev, pt_curr):
    global statex, statey, Projection_mat_homo
    # steo 1 is to cimpute the Essential matrix
    E = K_calib.T.dot(F.dot(K_calib))
    # we will find svd of E next
    U, S,VT = svd(E)

    # now we need to see if we can extract R and t from here
    points, R, t, mask = cv2.recoverPose(E, pt_prev, pt_curr)
    print("R is {}".format(R))
    print("t is {}".format(t))

    # now I have my R and t, I need to plot the trajectory
    Projection_mat = np.hstack((R,t))
    Projection_mat = np.vstack((Projection_mat,[0,0,0,1]))
    Projection_mat_homo = Projection_mat_homo.dot(Projection_mat)

    #lets plot the last column which is the T
    # print(Projection_mat_homo)
    x = Projection_mat_homo[0,3] #/ Projection_mat_homo[2,3]
    y = Projection_mat_homo[1,3] #/ Projection_mat_homo[2,3]
    statex.append(x)
    statey.append(y)

    sintht = Projection_mat_homo[1,0]
    costht = Projection_mat_homo[0,0]
    theta = m.atan2(sintht, costht)

    state_theta.append(m.degrees(theta))
    print(Projection_mat_homo)

def plot_side_by_side(prev_frame, cv_image, good_old, good_new):
    #we know that image width is 640, so every pixel
    if(good_old.shape[0]>8):
        fin_image = cv2.hconcat([prev_frame, cv_image])

        # creating mask
        mask = np.zeros_like(fin_image)
        #draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(mask, (int(c),int(d)), (int(a+640),int(b)),color[i].tolist(), 2)
            fin_image = cv2.circle(fin_image, (int(c),int(d)), 5, color[i].tolist(), -1)
        fin_image = cv2.add(fin_image, mask)

        cv2.imshow("combined", fin_image)
        cv2.waitKey(3)

def plot_all(prev_frame, cv_image, pt_prev, pt_curr):
    fin_image = cv2.hconcat([prev_frame, cv_image])
    # creating mask
    mask = np.zeros_like(fin_image)
    #draw the tracks
    for i, (new, old) in enumerate(zip(pt_curr, pt_prev)):
        a, b = new.ravel()
        c, d = old.ravel()
        mask = cv2.line(mask, (int(c),int(d)), (int(a+640),int(b)),color[i].tolist(), 2)
        fin_image = cv2.circle(fin_image, (int(c),int(d)), 5, color[i].tolist(), -1)
    fin_image = cv2.add(fin_image, mask)

    cv2.imshow(" all", fin_image)
    cv2.waitKey(3)

def image_cb(msg):
    global ini_image_obtained, prev_frame, Projection_mat_homo, statex, statey, state_theta, origx , origy
    """
    need to store prev frame
    """
    try:
        cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    if not ini_image_obtained:
        ini_image_obtained = True
    else:
        # cv2.imshow("Prev frame", prev_frame)
        # we will take the first frame and find corners in it
        gray_prev = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
        gray_current = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        pt_prev = cv2.goodFeaturesToTrack(gray_prev, mask = None, **feature_params)

        #create a mask for drawing purpose
        mask = np.zeros_like(prev_frame)
        #lets calculate optical flow now
        pt_curr, st, err = cv2.calcOpticalFlowPyrLK(gray_prev, gray_current, pt_prev, None, **lk_params)

        #selet good points
        good_new = pt_curr[st==1]
        good_old = pt_prev[st==1]
        """
        plot images side by side
        """
        # plot_side_by_side(prev_frame, cv_image, good_old, good_new)
        plot_all(prev_frame, cv_image, pt_prev, pt_curr)

        if good_new.shape[0] > 8:
            """
            this means that although I am going to capture transformation between the two frames
            I wont be getting enough features every time
            so I am basically skipping some frames
            """
            F = findFundamentalMatrix2(pt_prev, pt_curr)
            extractRtFromF(F, pt_prev, pt_curr)
        else:
            """
            I need to update based on imu data and/or odom data
            """
            # imu_data = rospy.wait_for_message('/pioneer2dx/imu', Imu)
            odom_data = rospy.wait_for_message('/pioneer2dx/odom', Odometry)
            x_vel = odom_data.twist.twist.linear.x
            yaw_rate = odom_data.twist.twist.angular.z
            #last Projection_mat_homo would be correct
            #need to extract x_prev, y_prev and theta_prev from it
            x_prev = Projection_mat_homo[0,3]
            y_prev = Projection_mat_homo[1,3]
            sintht = Projection_mat_homo[1,0]
            costht = Projection_mat_homo[0,0]
            theta_prev = m.atan2(sintht, costht)
            #
            x,y,theta = update(x_prev, y_prev, theta_prev, x_vel, yaw_rate)
            Projection_mat_homo = np.array([[m.cos(theta),-m.sin(theta),0,x],
                                                                  [m.sin(theta),m.cos(theta),0,y],
                                                                  [0,0,1,1],
                                                                  [0,0,0,1]])
            statex.append(x)
            statey.append(y)
            state_theta.append(theta)

        #now update the previous frame and previous points
        gray_prev = gray_current.copy()
        pt_prev = good_new.reshape(-1, 1, 2)
    prev_frame =  cv_image
    odom_data = rospy.wait_for_message('/pioneer2dx/odom', Odometry)
    origx.append(odom_data.pose.pose.position.x)
    origy.append(odom_data.pose.pose.position.y)


def camera_feed_sub():
    rospy.Subscriber('/pioneer2dx/pioneer2dx/camera/image_raw',Image,image_cb)

if __name__ == "__main__":
    try:
        camera_feed_sub()
        rospy.spin()
        plt.plot(statex,statey,'c-')
        plt.plot(origx, origy,'g-')
        plt.show()
    except rospy.ROSInterruptException:
        pass
