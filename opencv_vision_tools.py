import cv2
import argparse
import numpy as np
import pyzbar.pyzbar as pyzbar

class BigSensorTool:
    def __init__(self, confThreshold=0.5, nmsThreshold=0.5, objThreshold=0.5):
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.objThreshold = objThreshold

    # opencv Blob分析演示
    def get_blob(srcimg,minThreshold, maxThreshold, minArea, maxArea ):
        ver = (cv2.__version__).split('.')
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = minThreshold
        params.maxThreshold = maxThreshold
        params.filterByArea = True
        params.minArea = minArea
        params.maxArea = maxArea
        if int(ver[0]) < 3:
          detector = cv2.SimpleBlobDetector()
        else:
          detector = cv2.SimpleBlobDetector_create()
                        
        # 检测blobs
        keypoints = detector.detect(srcimg)
                        
        # 用红色圆圈画出检测到的blobs
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS 确保圆的大小对应于blob的大小
        im_with_keypoints = cv2.drawKeypoints(srcimg, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        return im_with_keypoints

    # opencv轮廓提取演示
    def get_edge(srcimg):
        srcimg = cv2.Canny(srcimg, 50, 100)
        return srcimg

    # opencv 二维码识别演示
    def get_qrcode(image):
        barcodes = pyzbar.decode(image)
        rects_list = []
        polygon_points_list = []
        QR_info = []

        # 这里循环，因为画面中可能有多个二维码
        for barcode in barcodes:
            # 提取条形码的边界框的位置
            # 画出图像中条形码的边界框
            (x, y, w, h) = barcode.rect
            rects_list.append((x, y, w, h))
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            polygon_points = barcode.polygon
            # print(f"polygon_points: {polygon_points}")  # polygon_points: [Point(x=217, y=174), Point(x=257, y=353), Point(x=433, y=316), Point(x=394, y=140)]
            # print(f"polygon_points: {polygon_points[0]}")  # polygon_points: Point(x=217, y=174)
            point_x, point_y = polygon_points[0]
            # print(f"point_x, point_y: {point_x, point_y}")  # point_x, point_y: (217, 174)

            extract_polygon_points = np.zeros((4, 2), dtype=np.int)
            for idx, points in enumerate(polygon_points):
                point_x, point_y = points  # 默认得到的point_x, point_y是float64类型
                extract_polygon_points[idx] = [point_x, point_y]

            print(extract_polygon_points.shape)  # (4, 2)

            # 不reshape成 (4,1 2)也是可以的
            extract_polygon_points = extract_polygon_points.reshape((-1, 1, 2))
            polygon_points_list.append(extract_polygon_points)

            # 要加上中括号，否则只会绘制四个点
            # cv2.polylines(image, extract_polygon_points, isClosed=True, color=(255, 0, 255), thickness=2)

            # 绘制多边形
            cv2.polylines(image, [extract_polygon_points], isClosed=True, color=(255, 0, 255), thickness=2,
                          lineType=cv2.LINE_AA)

            # 条形码数据为字节对象，所以如果我们想在输出图像上画出来，就需要先将它转换成字符串
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type

            # 绘出图像上条形码的数据和条形码类型
            text = "{} ({})".format(barcodeData, barcodeType)
            QR_info.append(text)
            cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (0, 0, 125), 2)

            # 向终端打印条形码数据和条形码类型
            print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        return image, rects_list, polygon_points_list, QR_info

    def display_qrcode(image, rects_list, polygon_points_list, QR_info):
        # 把检测到二维码的信息再绘制到BGR彩色图像上
        for data in zip(rects_list, polygon_points_list, QR_info):
            print(f"data: {data}")
            x, y, w, h = data[0]
            polygon_points = data[1]
            text = data[2]
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.polylines(image, [polygon_points], isClosed=True, color=(255, 0, 255), thickness=2,
                          lineType=cv2.LINE_AA)
            cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (0, 0, 125), 2)
        return image
