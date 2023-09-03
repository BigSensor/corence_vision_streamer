#!/usr/bin/env python
# -*- coding:utf-8 -*-
#mjpg-steamer服务端
#1.　先通过在服务器端利用OpenCV捕获到视频的每一帧图片
#2.　将这些图片进行压缩成JPEG格式，这样能减小图片大小，便于传输
#3.　按照提前协商好的分辨率和帧数进行打包编码传输
#4.　利用服务器端打开端口8080，建立mjpg-steamer服务
#5.  上位机用浏览器访问http://192.168.137.230:8080/stream.mjpg,即可预览相机图片

import logging
import socketserver
from threading import Condition, Thread
from PIL import Image
import cv2
import traceback
import io
import time
import os
import urllib
from urllib.parse import urlparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import numpy as np
import threading
import xml.etree.ElementTree as ET
from corence_vision_tools import BigSensorTool as bst
import platform

if ('Linux' == platform.system()):
    import RPi.GPIO as GPIO

m_start_ms=0
m_grab_count=0
m_output_holding=0
LINE0_PIN_NUM =  11
LINE1_PIN_NUM =  13
LINE2_PIN_NUM =  3
LINE3_PIN_NUM =  5

def gpio_init():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LINE0_PIN_NUM,GPIO.IN)
    GPIO.setup(LINE1_PIN_NUM,GPIO.IN)
    GPIO.setup(LINE2_PIN_NUM,GPIO.OUT)
    GPIO.setup(LINE3_PIN_NUM,GPIO.OUT)

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, frame):
        with self.condition:
            self.frame = frame
            self.condition.notify_all()


class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        print("get:",self.path)
        # 自定义网站访问地址，修改self.path，默认http://ip:8080/stream.mjpg
        if self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                traceback.print_exc()
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
                
        elif self.path.endswith('.jpg'):
                self.send_response(200)
                self.send_header('Pragma:', 'no-cache')
                self.send_header('Cache-Control:', 'no-cache')
                self.send_header('Content-type:','image/jpeg')
                self.end_headers()
                img = self.container.finalq.get()
                ret, img_jpg = cv2.imencode(".jpeg", img, (cv2.IMWRITE_JPEG_QUALITY, 90))
                # self.send_header('Content-length:', str(len(img_jpg)))
                img_str = img_jpg.tostring()
                self.wfile.write(img_str)
                
        elif self.path.endswith('.html') or self.path == "/":
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write('<html><head></head><body>')
            self.wfile.write('<img src="http://localhost:8080/stream.mjpg" height="240px" width="320px"/>')
            self.wfile.write('</body></html>')
            return
        else:
            self.send_error(404)
            self.end_headers()

    def do_POST(self):
        print(self.path)
        # 自定义网站访问地址，修改self.path，默认http://ip:8080/stream.mjpg
        parsed_path = urlparse(self.path) #'https://www.example.com:80/path/to/file/?query=string#anchor'
        print(parsed_path.scheme)     # 输出："https"
        print(parsed_path.netloc)     # 输出："www.example.com:80"
        print(parsed_path.path)       # 输出："/path/to/file/"
        print(parsed_path.query)      # 输出："query=string"
        print(parsed_path.fragment)   # 输出："anchor"
        try:
            params = dict([p.split('=') for p in parsed_path[4].split('&')])
            print(params)
        except:
            params = {}

##    def do_POST(self):
##        print("post:",self.path)
##        mpath,margs=urllib.splitquery(self.path)
##        datas = self.rfile.read(int(self.headers['content-length']))
##        self.do_action(mpath, datas)

    def do_action(self, path, args):
        self.outputtxt(path + args )

    def outputtxt(self, content):
        #指定返回编码
        enc = "UTF-8"
        content = content.encode(enc)
        f = io.BytesIO()
        f.write(content)
        f.seek(0)
        self.send_response(200)
        self.send_header("Content-type", "text/html; charset=%s" % enc)
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        shutil.copyfileobj(f,self.wfile)


class StreamingServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class Camera:
    def __init__(self, output, width, height, framerate, url):
        self.output = output
        self.width = width
        self.height = height
        self.framerate = framerate
        self.url = url

    def __enter__(self):
        # 相机或rtsp流打开路径，参数0表示打开笔记本的内置摄像头

        self.cap = cv2.VideoCapture(self.url)
        #if(self.width >= 640):
            #self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))  # 视频流格式
            #self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('B', 'G', 'R', '3'))  # 视频流格式
            #self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('R', 'G', 'B', '3'))  # 视频流格式
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.stop_capture = False
        self.thread = Thread(target=self.capture)
        self.thread.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop_capture = True
        self.thread.join()
        self.cap.release()

    def getIniVal(self, secname, keyname, defVal='', inifile='.\\para.ini'):
        alines = open(inifile, 'r', errors='ignore').readlines()
        findsec = 0
        for linec in alines:
            if linec.find('[' + secname + ']') >= 0:
                findsec = 1
            if linec.find(keyname + '=') >= 0 and findsec == 1:
                return linec[len(keyname) + 1:].strip()
        return defVal

    def capture(self):
        # 实例化一个BigSensorTool对象
        global m_start_ms
        global m_grab_count
        global m_output_holding
        frame_duration = 1. / self.framerate
        while not self.stop_capture:
            start = time.time()
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                #### example 1 get_edge
                #srcimg = bst.get_edge(frame)

                #### example 2 get_qrcode
                #srcimg, rects_list, polygon_points_list, QR_info = bst.get_qrcode(frame)
                #srcimg = bst.display_qrcode(srcimg, rects_list, polygon_points_list, QR_info)
                if m_start_ms ==0:
                    m_start_ms = time.time_ns()/1000

                #### example 3 get_blob
                if m_grab_count%10==0:
                    roi_box_list,roi_range_list,cv_type,OutputSource,OutputHoldTime = bst.get_camera_xml(0)
                m_grab_count+=1
                if m_grab_count>100000:
                    m_grab_count = 0
                rects_list=[]
                areaes_list=[]
                  # 颜色阈值下界(HSV) lower boudnary
                ColorLower = (0, 0, 0)
                  # 颜色阈值上界(HSV) upper boundary
                ColorUpper = (0, 0, 0)
                color_mode = cv2.COLOR_RGB2HSV
                min_area=100
                max_area=99999
                x = 0
                y = 0
                w = frame.shape[1]
                h = frame.shape[0]
                for roi_index,roi_box in enumerate(roi_box_list):
                    roi_range = ((0, 0, 0), (0, 0, 0), (10, 99999))#default
                    if roi_index<len(roi_range_list):
                        roi_range=roi_range_list[roi_index]
                    ColorLower = roi_range[0]  # 100,130,50
                    ColorUpper = roi_range[1]  # 200,200,130
                    min_area = roi_range[2][0]
                    max_area = roi_range[2][1]
                    #no roi
                    if roi_box[2] ==0 or roi_box[3] ==0:
                        roi_box[2] = frame.shape[1]
                        roi_box[3] = frame.shape[0]

                    x = roi_box[0]
                    y = roi_box[1]
                    w = roi_box[2]
                    h = roi_box[3]
                    #cv2.cvtColor(frame, frame, cv2.COLOR_RGB2Lab)
                    #roi_image = frame[y:y + h, x:x + w]
                    #cv2.imshow('frame', frame)          #展示图片
                    #min_area=100
                    #max_area=99999
                    # 颜色阈值下界(HSV) lower boudnary
                    #ColorLower = (96, 210, 85)
                    # 颜色阈值上界(HSV) upper boundary
                    #ColorUpper = (114, 255, 231)
                    #find_color_blobs(img, Roi=None, lowerColor, upperColor, ColorMode, min_Area=0, max_Area=None)
                    if cv_type == 1:
                        color_mode = cv2.COLOR_RGB2Lab#COLOR_RGB2Lab
                    else:
                        color_mode = cv2.COLOR_RGB2HSV
                    #print("cv_type",cv_type)
                    rects,areaes = bst.find_color_blobs(frame, roi_box, ColorLower, ColorUpper, color_mode, min_area,max_area)

                    rects_list.extend(rects)
                    areaes_list.extend(areaes)
                #判断是否检出对象
                detected = len(rects_list)
                #如果有对象就输出IO并开始计数
                if detected>0 and m_output_holding ==0:
                    m_start_ms = time.time_ns() / 1000000
                    print(OutputSource,"On",OutputHoldTime)
                    if ('Linux' == platform.system()):
                        GPIO.output(LINE2_PIN_NUM,True)
                    m_output_holding=1

                current_ms = time.time_ns() / 1000000
                # 如果有IO输出并持续倒保持时间，后如果未检出对象就IO置零
                if m_output_holding>0 and (current_ms-m_start_ms)>OutputHoldTime:
                    if detected == 0:
                        print(OutputSource, "Off", current_ms-m_start_ms)
                        if ('Linux' == platform.system()):
                            GPIO.output(LINE2_PIN_NUM,False)
                        m_output_holding=0
                #print("cv_type,max_area:",cv_type,max_area)
                if max_area < 1:
                    roi_image = frame[y:y + h, x:x + w]
                    # 转换色彩空间 HSV
                    img_hsv = cv2.cvtColor(roi_image, color_mode)  # cv2.COLOR_RGB2HSV
                    # 根据颜色阈值转换为二值化图像
                    mask = cv2.inRange(img_hsv, ColorLower, ColorUpper)
                    mask = cv2.erode(mask, None, iterations=2)
                    mask = cv2.dilate(mask, None, iterations=2)
                    frame = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
                # 绘制色块的矩形区域
                srcimg = bst.draw_color_blob_rect(frame, rects_list, areaes_list)
                if ('Windows' == platform.system()):
                    # 在HighGUI窗口 展示最终结果 更新画面
                    cv2.namedWindow('result', flags=cv2.WINDOW_NORMAL | cv2.WINDOW_FREERATIO)
                    cv2.imshow('result', srcimg)

                img = Image.fromarray(srcimg)
                img.save(self.output, format='JPEG')
            elapsed = time.time() - start
            logging.debug("Frame acquisition time: %.2f" % elapsed)
            if elapsed < frame_duration:
                time.sleep(frame_duration - elapsed)

try:
    if ('Linux' == platform.system()):
        gpio_init()
    output = StreamingOutput()
    # 可以修改视频的fps，宽度和高度
    with Camera(output, 640, 480, 25, url=0) as camera:
        # 访问地址和端口
        address = ('', 8080)
        server = StreamingServer(address, StreamingHandler)
        server.serve_forever()
except KeyboardInterrupt:
    pass
