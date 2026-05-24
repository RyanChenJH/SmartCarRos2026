import cv2
import numpy as np

def load_image(image_path):
    """加载图像文件"""
    return cv2.imread(image_path)

def save_image(image, output_path):
    """保存图像文件"""
    cv2.imwrite(output_path, image)

def resize_image(image, width, height):
    """调整图像大小"""
    return cv2.resize(image, (width, height))

def convert_to_grayscale(image):
    """转换为灰度图"""
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 直接使用函数，不需要导入
if __name__ == "__main__":
    # 加载图像
    img = load_image("red_light_detected.jpg")
    
    # 检查图像是否加载成功
    if img is not None:
        # 转换为灰度
        gray_img = convert_to_grayscale(img)
        # 保存结果
        save_image(gray_img, "output_gray.jpg")
        print("图像处理完成！")
    else:
        print("图像加载失败，请检查文件路径")