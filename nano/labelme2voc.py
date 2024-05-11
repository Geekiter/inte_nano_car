#!/usr/bin/env python

from __future__ import print_function

import argparse
import glob
import os
import sys

import imgviz
import labelme
import os.path as osp

try:
    import lxml.builder
    import lxml.etree
except ImportError:
    print("Please install lxml:\n\n    pip install lxml\n")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("input_dir", help="input annotated directory")
    parser.add_argument("output_dir", help="output dataset directory")
    parser.add_argument("--labels", help="labels file", required=True)
    parser.add_argument("--noviz", help="no visualization", action="store_true")
    args = parser.parse_args()

    if osp.exists(args.output_dir):
        print("Output directory already exists:", args.output_dir)
        sys.exit(1)
    os.makedirs(args.output_dir)
    os.makedirs(osp.join(args.output_dir, "JPEGImages"))
    os.makedirs(osp.join(args.output_dir, "Annotations"))
    os.makedirs(osp.join(args.output_dir, "ImageSets", "Main"))
    if not args.noviz:
        os.makedirs(osp.join(args.output_dir, "AnnotationsVisualization"))
    print("Creating dataset:", args.output_dir)

    labels = []
    class_name_to_id = {}
    for i, line in enumerate(open(args.labels).readlines()):
        class_id = i - 1  # starts with -1
        class_name = line.strip()
        class_name_to_id[class_name] = class_id
        if class_id == -1:
            assert class_name == "__ignore__"
            continue
        elif class_id == 0:
            assert class_name == "BACKGROUND"
            # continue
        labels.append(class_name)
    print(labels)
    labels = tuple(labels)
    print("labels:", labels)
    out_labels_file = osp.join(args.output_dir, "labels.txt")
    with open(out_labels_file, "w") as f:
        f.writelines("\n".join(labels))
    print("Saved labels:", out_labels_file)

    for filename in glob.glob(osp.join(args.input_dir, "*.json")):
        print("Generating dataset from:", filename)

        label_file = labelme.LabelFile(filename=filename)

        base = osp.splitext(osp.basename(filename))[0]
        out_img_file = osp.join(args.output_dir, "JPEGImages", base + ".jpg")
        out_xml_file = osp.join(args.output_dir, "Annotations", base + ".xml")
        if not args.noviz:
            out_viz_file = osp.join(
                args.output_dir, "AnnotationsVisualization", base + ".jpg"
            )

        img = labelme.utils.img_data_to_arr(label_file.imageData)
        imgviz.io.imsave(out_img_file, img)

        maker = lxml.builder.ElementMaker()
        xml = maker.annotation(
            maker.folder(),
            maker.filename(base + ".jpg"),
            maker.database(),  # e.g., The VOC2007 Database
            maker.annotation(),  # e.g., Pascal VOC2007
            maker.image(),  # e.g., flickr
            maker.size(
                maker.height(str(img.shape[0])),
                maker.width(str(img.shape[1])),
                maker.depth(str(img.shape[2])),
            ),
            maker.segmented(),
        )

        bboxes = []
        labels = []
        for shape in label_file.shapes:
            print(label_file.shapes)
            if shape["shape_type"] != "rectangle":
                print(
                    "Skipping shape: label={label}, " "shape_type={shape_type}".format(
                        **shape
                    )
                )
                continue

            class_name = shape["label"]
            print(labels)
            class_id = class_name.index(class_name)

            (xmin, ymin), (xmax, ymax) = shape["points"]
            # swap if min is larger than max.
            xmin, xmax = sorted([xmin, xmax])
            ymin, ymax = sorted([ymin, ymax])

            bboxes.append([ymin, xmin, ymax, xmax])
            labels.append(class_id)

            xml.append(
                maker.object(
                    maker.name(shape["label"]),
                    maker.pose(),
                    maker.truncated(),
                    maker.difficult(),
                    maker.bndbox(
                        maker.xmin(str(xmin)),
                        maker.ymin(str(ymin)),
                        maker.xmax(str(xmax)),
                        maker.ymax(str(ymax)),
                    ),
                )
            )

        if not args.noviz:
            captions = [labels[label] for label in labels]
            viz = imgviz.instances2rgb(
                image=img,
                labels=labels,
                bboxes=bboxes,
                captions=captions,
                font_size=15,
            )
            imgviz.io.imsave(out_viz_file, viz)

        with open(out_xml_file, "wb") as f:
            f.write(lxml.etree.tostring(xml, pretty_print=True))
    # 遍历JEPEGImages文件夹，前80%的文件名写入train.txt，后20%的文件名写入trainval.txt
    # 生成train.txt
    train_len = int(len(os.listdir(osp.join(args.output_dir, "JPEGImages"))) * 0.8)
    trainval_len = len(os.listdir(osp.join(args.output_dir, "JPEGImages"))) - train_len
    with open(osp.join(args.output_dir, "ImageSets", "Main", "train.txt"), "w") as f:
        # 取前80%的文件名
        for filename in os.listdir(osp.join(args.output_dir, "JPEGImages"))[:train_len]:
            f.write(osp.splitext(filename)[0] + "\n")
    # 生成trainval.txt
    with open(osp.join(args.output_dir, "ImageSets", "Main", "test.txt"), "w") as f:
        # 取后20%的文件名
        for filename in os.listdir(osp.join(args.output_dir, "JPEGImages"))[train_len:]:
            f.write(osp.splitext(filename)[0] + "\n")
    with open(osp.join(args.output_dir, "ImageSets", "Main", "trainval.txt"), "w") as f:
        # 取后20%的文件名
        for filename in os.listdir(osp.join(args.output_dir, "JPEGImages"))[train_len:]:
            f.write(osp.splitext(filename)[0] + "\n")


if __name__ == "__main__":
    main()
