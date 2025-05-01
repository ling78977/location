import cv2
import numpy as np
import matplotlib.pyplot as plt

def main():
    # 读取PGM图像
    image_path = '/home/ling/rm/location/src/my_nav/maps/first_map2.pgm'  # 替换为你的PGM图像路径
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # 检查图像是否成功读取
    if image is None:
        print("Error: Could not read the image.")
        return

    # 创建结构元素
    kernel = np.ones((4, 4), np.uint8)  # 5x5的矩形结构元素

    # 进行开操作
    opening = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
    cv2.imshow("dddd",opening)
    cv2.waitKey(0)

if __name__ == "__main__":
    main()
