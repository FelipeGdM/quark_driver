import time

import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np

from .models.experimental import attempt_load
from .utils.general import check_img_size, non_max_suppression, scale_coords, set_logging
from .utils.plots import plot_one_box
from .utils.torch_utils import select_device, time_synchronized, TracedModel
from .utils.datasets import letterbox

class Yolov7:
    def __init__(self, weights_path, conf_thres, iou_thres, img_size, classes) -> None:
        self.weights = weights_path
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.img_size = img_size
        self.classes = classes

        self.startup()

    def startup(self):
        # Initialize
        set_logging()
        self.device = select_device('0')
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # self.webcam = self.source.isnumeric() or self.source.endswith('.txt') or self.source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
        self.webcam = True

        # Load model
        self.model = attempt_load(self.weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(self.img_size, s=self.stride)  # check img_size

        self.old_img_w = self.old_img_h = imgsz
        self.old_img_b = 1

        self.model = TracedModel(self.model, self.device, self.img_size)

        if(self.half):
            self.model.half()

        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once

        t0 = time.time()

    def detect(self, imgs):
        img0 = imgs.copy()

        # Letterbox
        img = [letterbox(x, self.img_size, auto=True, stride=self.stride)[0] for x in img0]
        print(f'Shape img: {np.shape(img)}, img0: {np.shape(img0)}')
        # Stack
        img = np.stack(img, 0)

        # Convert
        img = img[:, :, :, ::-1].transpose(0, 3, 1, 2)  # BGR to RGB, to bsx3x416x416
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0    
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup
        if self.device.type != 'cpu' and (self.old_img_b != img.shape[0] or self.old_img_h != img.shape[2] or self.old_img_w != img.shape[3]):
            self.old_img_b = img.shape[0]
            self.old_img_h = img.shape[2]
            self.old_img_w = img.shape[3]
            for i in range(3):
                self.model(img)[0]

        # Inference
        t1 = time_synchronized()
        pred = self.model(img)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes)
        t3 = time_synchronized()
        centers = []

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if self.webcam:  # batch_size >= 1
                s, im0 = '%g: ' % i, img0[i].copy()
            else:
                s, im0 = '', img0

            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    centers.append((xyxy, conf))
                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=1)

            # Print time (inference + NMS)
            print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')
        
        return im0, centers
