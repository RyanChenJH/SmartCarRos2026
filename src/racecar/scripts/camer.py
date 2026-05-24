import cv2
def open_camera():
    """直接打开摄像头画面"""
    # 尝试不同的摄像头后端
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("错误: 无法打开摄像头")
        return
    frame_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("警告: 无法读取摄像头帧")
            continue
        frame_count += 1
        
        
        # 显示画面
        cv2.imshow('Camera', frame)
        
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    print("摄像头已关闭")

# 使用示例
if __name__ == "__main__":
    open_camera()