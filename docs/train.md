# get data

>In nano

```shell
python3 get_video_imgs.py
```

# label data

> In local machine

```shell
pip3 install labelme
```

# convert data format from labelme to yolo
> In local machine
```shell
pip3 install labelme2yolo
labelme2yolo --json_dir /path/to/labelme_json_dir/
```

# train
> In local machine
## install requirements

```shell
pip3 install -r requirements.txt
```

## train model

```shell
cd yolov5
python train.py --data coco.yaml --epochs 300 --weights '' --cfg yolov5n.yaml  --batch-size 128
                                                                 yolov5s                    64
                                                                 yolov5m                    40
                                                                 yolov5l                    24
```

### my train command

```shell
cd yolov5
python train.py --data "C:\Users\alber\dev\py\inte_nano_car\nano\video_images\images_2\YOLODataset\dataset.yaml" --epochs 100 --weights yolov5n.pt --cfg models/yolov5n.yaml  --batch-size 1
```

copy the best.pt file to the nano in the path ./runs/train/exp/weights/best.pt

rename the file to yolov5_best.pt

# run

> In nano

```shell
pip3 install -r requirements.txt
python3 main.py
```




