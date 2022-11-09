import time

import torch
import torch.backends.cudnn as cudnn
from numpy import random

from .models.experimental import attempt_load
from .utils.general import check_img_size, non_max_suppression, scale_coords, set_logging
from .utils.plots import plot_one_box
from .utils.torch_utils import select_device, time_synchronized, TracedModel

class Yolov7:
    def __init__(self, weights_path, conf_thres, iou_thres, img_size, classes) -> None:
        self.weights = weights_path
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.img_size = img_size
        self.classes = classes
        # self.model
        # self.half
        # self.device
        # self.names
        # self.colors

        self.startup()

    def startup(self):
        # Initialize
        set_logging()
        self.device = select_device('0')
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(self.weights, map_location=self.device)  # load FP32 model
        stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(self.img_size, s=stride)  # check img_size

        self.model = TracedModel(self.model, self.device, self.img_size)

        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once

        t0 = time.time()

    def detect(self, img):
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0    
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        im0 = img.copy()

        # Warmup
        if self.device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
            old_img_b = img.shape[0]
            old_img_h = img.shape[2]
            old_img_w = img.shape[3]
            for i in range(3):
                self.model(img)[0]

        # Inference
        t1 = time_synchronized()
        pred = self.model(img)[0]
        t2 = time_synchronized()

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes)
        t3 = time_synchronized()

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if self.webcam:  # batch_size >= 1
                s, im0 = '%g: ' % i, im0[i].copy()
            else:
                s, im0 = '', im0

            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=1)

            # Print time (inference + NMS)
            print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')
        
        return im0
