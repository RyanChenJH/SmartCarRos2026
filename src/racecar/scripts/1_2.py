import cv2
import numpy as np
import glob
import os

class CameraCalibration:
    def __init__(self, pattern_size=(9, 6)):
        self.pattern_size = pattern_size
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.camera_matrix = None
        self.distortion_coeffs = None
        
    def capture_calibration_images(self, num_images=20):
        """采集标定图像"""
        # 尝试不同的摄像头后端
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        
        if not cap.isOpened():
            cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("错误: 无法打开摄像头")
            return
        
        # 优化摄像头参数
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 关键：减少缓冲区
        
        count = 0
        
        if not os.path.exists('calibration_images'):
            os.makedirs('calibration_images')
        
        print("摄像头已打开，开始采集标定图像...")
        print("操作说明：")
        print("- 将棋盘格放在摄像头前，确保能检测到角点")
        print("- 按空格键保存图像")
        print("- 按'q'键退出")
        
        while count < num_images:
            ret, frame = cap.read()
            if not ret:
                print("警告: 无法读取摄像头帧")
                continue
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_chess, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
            
            if ret_chess:
                cv2.drawChessboardCorners(frame, self.pattern_size, corners, ret_chess)
                cv2.putText(frame, f'Press SPACE to capture ({count}/{num_images})', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, 'Chessboard detected!', 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(frame, 'No chessboard detected', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(frame, f'Images captured: {count}/{num_images}', 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow('Calibration', frame)
            
            key = cv2.waitKey(30) & 0xFF  # 保持30毫秒
            if key == ord(' ') and ret_chess:
                cv2.imwrite(f'calibration_images/img_{count:02d}.jpg', frame)
                count += 1
                print(f"已保存 {count}/{num_images} 张图像")
            elif key == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
    
    def calibrate(self):
        """执行标定"""
        # 准备对象点
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        
        objpoints = []
        imgpoints = []
        
        images = glob.glob('calibration_images/*.jpg')
        
        if len(images) == 0:
            print("错误: 未找到标定图像，请先运行采集程序")
            return None, None
        
        print(f"找到 {len(images)} 张标定图像，开始处理...")
        
        for i, fname in enumerate(images):
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
            
            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                imgpoints.append(corners2)
                print(f"处理完成: {i+1}/{len(images)}")
        
        if len(objpoints) == 0:
            print("错误: 没有找到有效的标定图像")
            return None, None
        
        print(f"使用 {len(objpoints)} 张有效图像进行标定...")
        
        # 执行标定
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, 
                                                           gray.shape[::-1], None, None)
        
        self.camera_matrix = mtx
        self.distortion_coeffs = dist
        
        # 保存结果
        np.savez('camera_calibration.npz', 
                camera_matrix=mtx, 
                distortion_coeffs=dist)
        
        print("标定完成！")
        print("相机内参矩阵:")
        print(mtx)
        print("\n畸变系数:")
        print(dist)
        
        return mtx, dist

def real_time_undistortion():
    """实时畸变矫正"""
    try:
        calibration_data = np.load('camera_calibration.npz')
        camera_matrix = calibration_data['camera_matrix']
        distortion_coeffs = calibration_data['distortion_coeffs']
    except FileNotFoundError:
        print("错误: 未找到标定文件，请先运行标定程序")
        return
    
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("错误: 无法打开摄像头")
        return
    
    # 优化摄像头设置
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 关键：减少缓冲区
    
    print("实时畸变矫正演示")
    print("按'q'键退出")
    
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        frame_count += 1
        
        # 每隔一帧处理一次，减少计算负载
        if frame_count % 2 == 0:
            # 实时矫正
            undistorted = cv2.undistort(frame, camera_matrix, distortion_coeffs)
            
            # 显示原图和矫正后的图
            combined = np.hstack((frame, undistorted))
            cv2.putText(combined, 'Original', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(combined, 'Undistorted', (frame.shape[1] + 10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            cv2.imshow('Original | Undistorted', combined)
        else:
            # 只显示原图
            cv2.imshow('Original | Undistorted', frame)
        
        if cv2.waitKey(30) & 0xFF == ord('q'):  # 改为30毫秒
            break
    
    cap.release()
    cv2.destroyAllWindows()

# 使用示例
if __name__ == "__main__":
    calibrator = CameraCalibration()
    
    print("选择操作：")
    print("1. 采集标定图像")
    print("2. 执行标定")
    print("3. 实时矫正演示")
    print("4. 完整流程")
    
    choice = input("请输入选择 (1-4): ")
    
    if choice == "1":
        calibrator.capture_calibration_images(20)
    elif choice == "2":
        calibrator.calibrate()
    elif choice == "3":
        real_time_undistortion()
    elif choice == "4":
        # 完整流程
        calibrator.capture_calibration_images(20)
        camera_matrix, distortion_coeffs = calibrator.calibrate()
        if camera_matrix is not None:
            real_time_undistortion()
    else:
        print("无效选择")