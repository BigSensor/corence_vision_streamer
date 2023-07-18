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
from http.server import BaseHTTPRequestHandler, HTTPServer
import numpy as np
import threading
from opencv_vision_tools import BigSensorTool as bst

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
        else:
            self.send_error(404)
            self.end_headers()


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

                #### example 3 get_blob
                srcimg = bst.get_blob(frame,50,255,100,99999)

                img = Image.fromarray(srcimg)
                img.save(self.output, format='JPEG')
            elapsed = time.time() - start
            logging.debug("Frame acquisition time: %.2f" % elapsed)
            if elapsed < frame_duration:
                time.sleep(frame_duration - elapsed)

try:
    output = StreamingOutput()
    # 可以修改视频的fps，宽度和高度
    with Camera(output, 640, 480, 25,url=0) as camera:
        # 访问地址和端口
        address = ('', 8080)
        server = StreamingServer(address, StreamingHandler)
        server.serve_forever()
except KeyboardInterrupt:
    pass
