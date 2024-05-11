1. 收集数据

使用nanocar下的cd /home/nvidia/nanocar/ &&python3 /home/nvidia/nanocar/get_video_img.py，收集照片，保存在 /home/nvidia/nanocar/video_images文件夹下。

```shell
cd /home/nvidia/nanocar/ &&python3 /home/nvidia/nanocar/get_video_img.py
```

2. 标注数据

下载labelImg，打开video_images文件夹，标注完数据，上传到nano里

3. 转换为voc格式的数据

创建labels.txt文件，上传到/home/nvidia/nanocar/下，内容如下：

```
__ignore__
BACKGROUND
duck

```

用nanocar下的python3 /home/nvidia/nanocar/labelme2voc.py，将标注好的数据转换为voc格式的数据

```shell
rm -rf /home/nvidia/nanocar/video_images_output && python3 /home/nvidia/nanocar/labelme2voc.py /home/nvidia/nanocar/video_images/images_5 /home/nvidia/nanocar/video_images_output --labels /home/nvidia/nanocar/labels.txt --noviz
```



4. 训练模型

cd /root/dev/py/jetson-inference/python/training/detection/ssd/ &&  python3 /root/dev/py/jetson-inference/python/training/detection/ssd/train_ssd.py --checkpoint-folder=/home/nvidia/nanocar/video_images_output --dataset-type=voc --data=/home/nvidia/nanocar/video_images_output --model-dir=/root/dev/py/jetson-inference/build/aarch64/bin/networks/SSD-Mobilenet-v2/ --batch-size=2 --epochs=2



5. 转换为onnx格式
```shell

cd /home/nvidia/nanocar/video_images_output/
python3 /root/dev/py/jetson-inference/python/training/detection/ssd/onnx_export.py --input /root/dev/py/jetson-inference/python/training/detection/ssd/models/test/mb1-ssd-Epoch-90-Loss-1.1715968946615856.pth

```

6. 开始预测，启动模型

/root/dev/py/jetson-inference/examples/socket_example/build/detectnet-socket --network=/home/nvidia/nanocar/video_images_output/ssd-mobilenet.onnx --input-blob=input_0 --output-cvg=scores --output-bbox=boxes

7. 启动主程序

python3 /home/nvidia/nanocar/main.py
