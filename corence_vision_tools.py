import cv2
import argparse
import numpy as np
import pyzbar.pyzbar as pyzbar
import xml.etree.ElementTree as ET
import os
import platform

class BigSensorTool:
    def __init__(self, confThreshold=0.5, nmsThreshold=0.5, objThreshold=0.5):
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.objThreshold = objThreshold

    def get_camera_xml(cfg_id):
        roi_box = [0, 0, 0, 0]
        roi_range = [(0, 0, 0), (0, 0, 0), (10, 99999)]
        mv_type = 1
        roi_box_list = []
        roi_range_list = []
        OutputSource = 'Line2'
        OutputHoldTime = 100
        try:
            in_file_path = '/home/camera%d.xml' % (cfg_id)
            if not os.path.exists(in_file_path):
                if not os.path.exists('camera%d.xml' % (cfg_id)):
                    return [], [], 1, "", 0
                else:
                    in_file_path = 'camera%d.xml' % (cfg_id)
            in_file = open(in_file_path)  # home
            tree = ET.parse(in_file)
            root = tree.getroot()
            TriggerMode = root.find('TriggerMode').text
            TriggerSource = root.find('TriggerSource').text
            GrabTime = int(root.find('GrabTime').text)
            OutputMode = root.find('OutputMode').text
            OutputSource = root.find('OutputSource').text
            OutputHoldTime = int(root.find('OutputHoldTime').text)

            xmlroi = root.find('roi')
            for xmlbox in xmlroi.iter('bndbox'):
                # xmlbox = xmlroi.find('bndbox')
                if xmlbox is not None:
                    roi_box = (
                    int(xmlbox.find('xmin').text), int(xmlbox.find('ymin').text), int(xmlbox.find('width').text),
                    int(xmlbox.find('height').text))
                    roi_box_list.append(roi_box)

            threshold = root.find('threshold')
            xml_type = threshold.find('type')
            if xml_type is None:
                mv_type = 1
            else:
                mv_type = int(xml_type.text)
            for object in threshold.iter('object'):
                # object = threshold.find('object')
                if object is not None:
                    thresh_lower = (
                    int(object.find('min0').text), int(object.find('min1').text), int(object.find('min2').text))
                    thresh_upper = (
                    int(object.find('max0').text), int(object.find('max1').text), int(object.find('max2').text))
                    range_area = (float(object.find('min3').text), float(object.find('max3').text))
                    roi_range = (thresh_lower, thresh_upper, range_area)
                    roi_range_list.append(roi_range)
            return roi_box_list, roi_range_list, mv_type, OutputSource, OutputHoldTime
        except:
            return roi_box_list, roi_range_list, mv_type, OutputSource, OutputHoldTime

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

    # Blob色块识别 返回矩形信息
    # img 彩色图像
    # Roi
    #lowerb = (96, 210, 85) # 颜色阈值下界(HSV) lower boudnary
    #upperb = (114, 255, 231) # 颜色阈值上界(HSV) upper boundary
    #ColorMode cv2.COLOR_BGR2HSV/cv2.COLOR_BGR2Lab
    def find_color_blobs(img, roi_box, lowerColor, upperColor, ColorMode,
                        min_Area=0, max_Area=None):
        try:
            #ver = (cv2.__version__).split('.')

            if roi_box is None:
                roi_box = (0, 0, img.shape[1], img.shape[0])
            roi_x = roi_box[0]
            roi_y = roi_box[1]
            roi_w = roi_box[2]
            roi_h = roi_box[3]
            roi_image = img[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
            # 转换色彩空间 HSV
            img_hsv = cv2.cvtColor(roi_image, ColorMode)#cv2.COLOR_RGB2HSV
            # 根据颜色阈值转换为二值化图像
            mask = cv2.inRange(img_hsv, lowerColor, upperColor)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            if ('Windows' == platform.system()):
                cv2.namedWindow('mask', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
                cv2.imshow('mask', mask)
        # 寻找轮廓（只寻找最外侧的色块）bimg,
            contours, hier = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # 声明画布 拷贝自img
            canvas = np.copy(roi_image)
            # 外接矩形区域集合
            all_rects = []
            all_areaes = []
            if max_Area is None:
                # 如果最大面积没有设定，就设定为图像的面积
                max_Area = roi_image.shape[1]*roi_image.shape[0]# width*height

            # 遍历所有的边缘轮廓集合
            for cidx, cnt in enumerate(contours):
                # 获取联通域的外界矩形
                (x, y, w, h) = cv2.boundingRect(cnt)
                area = cv2.contourArea(cnt)  # 计算轮廓的面积
                if area >= min_Area and area <= max_Area:
                    # 将矩形的信息(tuple)添加到rects中
                    all_rects.append((roi_x+x, roi_y+y, w, h))
                    all_areaes.append(area)
            return all_rects, all_areaes
        except:
            return [],[]

    def draw_color_blob_rect(img, rects, areaes, color=(0, 0, 255)):
        '''
        绘制色块的矩形区域
        '''
        # 声明画布(canvas) 拷贝自img
        canvas = np.copy(img)
        # 遍历矩形区域
        for ind, rect in enumerate(rects):
            (x, y, w, h) = rect
            # 在画布上绘制矩形区域（红框）
            cv2.rectangle(canvas, pt1=(x, y), pt2=(x + w, y + h), color=color, thickness=3)
            if(ind < len(areaes)):
                text ="{}".format(int(areaes[ind]))
                font = cv2.FONT_HERSHEY_COMPLEX
                # 字体标注的位置， 内容，字体设置
                cv2.putText(canvas, text, (x + 32, y + 32), font, 1, (255, 0, 0), 1)

        return canvas
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
