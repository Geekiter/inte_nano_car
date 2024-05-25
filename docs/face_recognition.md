# 1. 获取数据集

```shell
python3 /home/nvidia/nanocar/get_pic.py
```

# 2. 重命名, 上传到目录

```shell
/home/nvidia/nanocar/face_known/
```

# 3. 获取zoom factor

修改/home/nvidia/nanocar/face_reco/main.py文件
```python
from nano import Nano

if __name__ == '__main__':
    nano = Nano()

    nano.cal_zf_from_kpu(real_dis=19)

```


```shell
python3 main_backup.py
```

# 4. 测试zoom factor

修改main.py文件
```python
from nano import Nano

if __name__ == '__main__':
    nano = Nano()

    nano.get_kpu_dis_by_zf(1.2)

```


```shell
python3 main_backup.py
```

# 5. 执行任务

修改main.py文件
```python
from nano import Nano

if __name__ == '__main__':
    nano = Nano()
    nano.run()

```

```shell
python3 main_backup.py
```