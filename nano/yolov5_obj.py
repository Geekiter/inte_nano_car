import sys
from pathlib import Path

yolov5_path = Path(__file__).parent / 'yolov5'

yolov5_path_str = str(yolov5_path.resolve())

if yolov5_path_str not in sys.path:
    sys.path.append(yolov5_path_str)

from yolov5.models.common import DetectMultiBackend
from yolov5.utils.general import (check_img_size, non_max_suppression, scale_boxes, xyxy2xywh)
from yolov5.utils.torch_utils import select_device
import torch
import numpy as np


class YOLOv5Detector:

    def __init__(self, weights, imgsz=(640, 640), conf_thres=0.25, iou_thres=0.45, max_det=1000, device='', dnn=False):
        self.device = select_device(device)
        self.model = DetectMultiBackend(weights, device=self.device, dnn=dnn)
        self.stride, self.names, _ = self.model.stride, self.model.names, self.model.pt
        self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.max_det = max_det
        self.model.warmup(imgsz=(1, 3, *self.imgsz))  # warmup

    def predict(self, frame):
        # Convert
        img = frame[..., ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.model.fp16 else img.float()  # uint8 to fp16/32
        img /= 255  # 0 - 255 to 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # expand for batch dim

        # Inference
        pred = self.model(img, augment=False, visualize=False)

        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=None, agnostic=False,
                                   max_det=self.max_det)

        # Process predictions
        det = pred[0]  # detections per image
        gn = torch.tensor(frame.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        # annotator = Annotator(frame, line_width=3, example=str(self.names))
        results = []
        if len(det):
            # Rescale boxes from img_size to frame size
            det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], frame.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det):
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                # Annotate image
                label = f'{self.names[int(cls)]}'  # annotator.box_label(xyxy, label, color=colors(int(cls), True))
                results.append((*xywh, conf, cls, label))

        return results
