from tello import Tello
import time
import av
import cv2
import math

tello = Tello()

tello.send_command('command')
begin = time.time()

while 1:
    end = time.time()
    diff = end-begin
    if diff > 1.0:
        tello.send_command('streamon')
        break

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
    #old_frame = video.decode()[0].to_nd_array(format = 'bgr24')
    count = 0
    flag1 = 1
    flag2 = 0
    for frame in video.decode():
        if count % 3 !=0:
            count += 1
            continue
        else:
            count = 0
            
            tello.send_command('forward 40')
            #tello.send_command('rc 0 0 0 0')
            time.sleep(0.04)
            img = frame.to_nd_array(format = 'bgr24')
            image1 = img
            image2 = img
            #print(img.shape)
            ############divide left and right##############
            left_img = image1[0:720,0:480]
            right_img = image2[0:720,480:960]
            #cv2.imshow("left",left_img)
            #cv2.imshow("right",right_img)
            ###############################################
            
            while flag1 == 1:
                old_frame = img
                frame1 = old_frame[0:720,0:480]
                frame2 = old_frame[0:720,480:960]
                
                #找到原始灰度图
                left_old_gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
                right_old_gray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
            
                #获取图像中的角点，返回到p0中
                left_p0 = cv2.goodFeaturesToTrack(left_old_gray, mask = None, **feature_params)
                right_p0 = cv2.goodFeaturesToTrack(right_old_gray, mask = None, **feature_params)
                
                flag1 = flag1 - 1
                
            ############calculate the optical flow value############
            left_frame_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
            right_frame_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
    
            # 计算光流
            left_p1, left_st, left_err = cv2.calcOpticalFlowPyrLK(left_old_gray, left_frame_gray, left_p0, None, **lk_params)
            right_p1, right_st, right_err = cv2.calcOpticalFlowPyrLK(right_old_gray, right_frame_gray, right_p0, None, **lk_params)
            
            # 选取好的跟踪点
            left_good_new = left_p1[left_st==1]
            left_good_old = left_p0[left_st==1]
            
            right_good_new = right_p1[right_st==1]
            right_good_old = right_p0[right_st==1]
            
            #计算特征点移动向量模长
            left_row = left_good_new.shape[0]
            right_row = right_good_new.shape[0]
            
            i = 0
            j = 0
            left_total_len = 0
            right_total_len = 0
            param1 = 1
            param2 = 1
            
            while i < left_row:
                delta_x = left_good_new[i][0] - left_good_old[i][0]
                delta_y = left_good_new[i][1] - left_good_old[i][1]
                left_total_len = left_total_len + math.sqrt((delta_x**2)+(delta_y**2))
                i = i+1
            if left_row != 0:
                left_ave_len = left_total_len / left_row
            
            while j < right_row:
                delta_x = right_good_new[j][0] - right_good_old[j][0]
                delta_y = right_good_new[j][1] - right_good_old[j][1]
                right_total_len = right_total_len + math.sqrt((delta_x**2)+(delta_y**2))
                j = j+1
            if right_row != 0:
                right_ave_len = right_total_len / right_row
            
            if right_ave_len != 0:
                param1 = left_ave_len / right_ave_len
            if left_ave_len != 0:
                param2 = right_ave_len / left_ave_len 
            
            if param1 >= 1.8 and flag2 == 0:
                #tello.send_command('rc 60 0 0 0')
                tello.send_command('right 20')
                time.sleep(0.02)
                #tello.send_command('rc 0 0 0 0')
                tello.send_command('stop')
                flag2 = 1
            elif param2 >= 1.8 and flag2 == 0:
                #tello.send_command('rc -60 0 0 0')
                tello.send_command('left 20')
                time.sleep(0.02)
                #tello.send_command('rc 0 0 0 0')
                tello.send_command('stop')
                flag2 = 1
            else:   
                #tello.send_command('forward 20')
                #tello.send_command('rc 0 0 0 0')
                time.sleep(0.02)
                flag2 = 0
            
            ########################################################
            
            cv2.imshow("Test", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # 更新上一帧的图像和追踪点
            left_old_gray = left_frame_gray.copy()
            left_p0 = left_good_new.reshape(-1,1,2)
            
            right_old_gray = right_frame_gray.copy()
            right_p0 = right_good_new.reshape(-1,1,2)
        
except Exception as e:
    print('fate error:{}'.format(e))

tello.send_command('streamoff')
tello.send_command('land')
cv2.destroyAllWindows()