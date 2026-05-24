import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture(0)  # 你的摄像头编号

cv2.namedWindow('Trackbars')

# 创建6个滑动条，分别对应HSV的上下限
cv2.createTrackbar('H_low', 'Trackbars', 0, 179, nothing)
cv2.createTrackbar('S_low', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('V_low', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('H_high', 'Trackbars', 179, 179, nothing)
cv2.createTrackbar('S_high', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('V_high', 'Trackbars', 255, 255, nothing)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.resize(frame, (320, 240))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 计算当前画面亮度均值
    v_mean = np.mean(hsv[:,:,2])

    # 根据亮度自动调整V_low和V_high
    # 你可以根据实际情况调整下面的映射关系
    auto_v_low = max(0, int(v_mean * 0.5))
    auto_v_high = min(255, int(v_mean * 1.2))

    # 用滑动条的值作为基础
    h_low = cv2.getTrackbarPos('H_low', 'Trackbars')
    s_low = cv2.getTrackbarPos('S_low', 'Trackbars')
    # v_low = cv2.getTrackbarPos('V_low', 'Trackbars')
    h_high = cv2.getTrackbarPos('H_high', 'Trackbars')
    s_high = cv2.getTrackbarPos('S_high', 'Trackbars')
    # v_high = cv2.getTrackbarPos('V_high', 'Trackbars')

    # 用自动调整的V阈值
    lower = np.array([h_low, s_low, auto_v_low])
    upper = np.array([h_high, s_high, auto_v_high])

    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('Original', frame)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    # 显示当前自动V阈值
    print(f"auto_v_low: {auto_v_low}, auto_v_high: {auto_v_high}")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()