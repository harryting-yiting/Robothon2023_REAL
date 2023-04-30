
import cv2
import numpy as np
def setHsv(event,x,y,flags,param):
    if event==cv2.EVENT_LBUTTONDOWN:
        print("HSV is",hsv_frame[y,x])
        cv2.setTrackbarPos('H_l','image', hsv_frame[y,x][0]-HSVvalue if (hsv_frame[y,x][0]-HSVvalue>0) else 0 )
        cv2.setTrackbarPos('H_h', 'image',hsv_frame[y,x][0]+HSVvalue if (hsv_frame[y,x][0]+HSVvalue<180) else 180)
        cv2.setTrackbarPos('S_l', 'image', hsv_frame[y,x][1]-HSVvalue if (hsv_frame[y,x][1]-HSVvalue>0) else 0 )
        cv2.setTrackbarPos('S_h', 'image', hsv_frame[y,x][1]+HSVvalue if (hsv_frame[y,x][1]+HSVvalue<255) else 255)
        cv2.setTrackbarPos('V_l', 'image', hsv_frame[y,x][2]-HSVvalue if (hsv_frame[y,x][2]-HSVvalue>0) else 0)
        cv2.setTrackbarPos('V_h', 'image', hsv_frame[y,x][2]+HSVvalue if (hsv_frame[y,x][2]+HSVvalue<255) else 255)

def nothing(x):
    pass
def createbars():
    cv2.createTrackbar("H_l","image",0,180,nothing)
    cv2.createTrackbar("H_h","image",0,180,nothing)
    cv2.createTrackbar("S_l","image",0,255,nothing)
    cv2.createTrackbar("S_h","image",0,255,nothing)
    cv2.createTrackbar("V_l","image",0,255,nothing)
    cv2.createTrackbar("V_h","image",0,255,nothing)

# cap =  cv2.VideoCapture(0)
# cap.set(3, 320)
# cap.set(4, 240)    
cv2.namedWindow("image")
lower = np.array([0,0,0])#设置初始值
upper = np.array([0,0,0])
HSVvalue=50

# Configure depth and color streams
# cap = cv2.VideoCapture(0)

createbars()
while True:
    # retval, color_frame=cap.read()
    # depth_frame = frames.get_depth_frame()
    # if not depth_frame or not color_frame:
        # continue
    # Convert images to numpy arrays
    # depth_image = np.asanyarray(depth_frame.get_data())
    # frame = np.array(color_frame)
    
    simg = cv2.imread('ex.jpg',cv2.IMREAD_UNCHANGED)
    x, y = simg.shape[0:2]
    frame = cv2.resize(simg, (int(y / 4), int(x / 4)))
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2LAB)#将图片由BGR颜色空间转化成HSV空间，HSV可以更好地分割颜色图形


    lower[0]=cv2.getTrackbarPos("H_l","image")#获取"H_l"滑块的实时值
    upper[0]=cv2.getTrackbarPos("H_h","image")#获取"H_h"滑块的实时值
    lower[1]=cv2.getTrackbarPos("S_l","image")
    upper[1]=cv2.getTrackbarPos("S_h","image")
    lower[2]=cv2.getTrackbarPos("V_l","image")
    upper[2]=cv2.getTrackbarPos("V_h","image")
    
    mask = cv2.inRange(hsv_frame,lower,upper)#cv2.inrange()函数通过设定的最低、最高阈值获得图像的掩膜
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.GaussianBlur(mask, (3, 3), 0)
    cv2.imshow("img",frame)
    cv2.setMouseCallback("img",setHsv)#点击屏幕中需要追踪的颜色 设置HSV大概的范围值
    cv2.imshow("mask",mask)


    
    conts,hier = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)#找出边界
    cv2.drawContours(frame,conts,-1,(0,255,0),3)#绘制轮廓
  
    #dst = cv2.bitwise_and(frame,frame,mask=mask)#对每一帧进行位与操作，获取追踪图像的颜色
    #cv2.imshow("dst",dst)

    if len(conts) >0 : #通过边缘检测来确定所识别物体的位置信息得到相对坐标
        conts = max(conts,key=cv2.contourArea)
        (x,y),radius = cv2.minEnclosingCircle(conts)
        cv2.circle(frame,(int(x),int(y)),int(radius),(255,0,255),2) #画出一个圆
        print(int(x),int(y))
    else:
        pass

    cv2.imshow("frame",frame)
    if cv2.waitKey(1)&0xff == 27:
        break
cap.release()
cv2.destroyAllWindows()
