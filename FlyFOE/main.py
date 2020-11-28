from tello import Tello
import time
import av
import cv2
import math
import numpy as np

tello = Tello()

tello.send_command('command')
begin = time.time()

time.sleep(1)
tello.send_command('streamon')

video = av.open('udp://0.0.0.0:11111','r')
index = 0

# ShiTomasi 角点检测参数
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

# lucas kanade光流法参数
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

tello.send_command('takeoff')
time.sleep(1)

try:
    # 获取第一帧，找到角点
    count = 0
    flag1 = 1
    flag2 = 0
    for frame in video.decode():
    #        if count % 5 !=0:
    #            count += 1
    #            continue
    #        else:
    #            count = 0
            
        tello.send_command('foward 60')
        tello.send_command('rc 0 0 0 0')
        img = frame.to_nd_array(format = 'bgr24')
        
        while flag1 == 1:
            old_frame = img
    
            #找到原始灰度图
            old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
        
            #获取图像中的角点，返回到p0中
            p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
            
            flag1 = flag1 - 1
            
        ############calculate the optical flow value############
        frame_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
        # 计算光流
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
        
        # 选取好的跟踪点
        good_new = p1[st==1]
        good_old = p0[st==1]
        
        #计算特征点移动向量模长
        row = good_new.shape[0]
        
        i = 0
        cnt = 0
        param = 480
        total_x = 0
        
        delta = good_new - good_old
        #delta[:,0],delta[:,1] = delta[:,1],delta[:,0]
        A = delta
        
        if np.linalg.det(A.T.dot(A)) != 0:
            #b = delta[:,0]*good_old[:,0] - delta[:,1]*good_old[:,1]
            b = delta[:,1]*good_old[:,0] - delta[:,0]*good_old[:,1]
            
            foe = np.dot(np.dot(np.linalg.inv(np.dot(A.T,A)),A.T),b)
            FOE = foe.reshape(1,2)
    
            vector_len = np.sqrt((delta[:,0]**2)+(delta[:,1]**2))
            distance_pixel_to_FOE = np.sqrt(((good_old[:,0] - FOE[0,0])**2)+((good_old[:,1] - FOE[0,1])**2))
            
            t = distance_pixel_to_FOE / vector_len
            
            row = t.shape[0]
            t = t.reshape(row,1)
            
            while i < row:
                if t[i,0] < 600:
                    total_x = total_x + good_old[i,0]
                    cnt = cnt + 1
                i = i + 1
            if cnt != 0:
                ave_x = total_x / cnt
                param = ave_x
        
        if param < 480 and flag2 == 0:
            tello.send_command('rc 40 0 0 0')
            time.sleep(0.02)
            tello.send_command('rc 0 0 0 0')
            flag2 = 1
        elif param > 480 and flag2 == 0:
            tello.send_command('rc -40 0 0 0')
            time.sleep(0.02)
            tello.send_command('rc 0 0 0 0')
            flag2 = 1
        else:   
            tello.send_command('rc 0 25 0 0')
            time.sleep(0.04)
            flag2 = 0
        
        ########################################################
        
        cv2.imshow("Test", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        # 更新上一帧的图像和追踪点
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1,1,2)
        
except Exception as e:
    print('fate error:{}'.format(e))

tello.send_command('streamoff')
tello.send_command('land')
cv2.destroyAllWindows()