#!/usr/bin/env python
import os
import random
import re

trainval_percent = 0.1
train_percent = 0.9
xmlfilepath = 'data/Annotations'
txtsavepath = 'data/ImageSets'
total_xml = os.listdir(xmlfilepath)

num = len(total_xml)
xml_list = [int(re.sub(r'\.xml', "", i)) for i in total_xml]
tv = int(num * trainval_percent)
tr = int(tv * train_percent)
trainval = random.sample(xml_list, tv)
train = random.sample(trainval, tr)

ftrainval = open('data/ImageSets/trainval.txt', 'w')
ftest = open('data/ImageSets/test.txt', 'w')
ftrain = open('data/ImageSets/train.txt', 'w')
fval = open('data/ImageSets/val.txt', 'w')

for i in range(num):
    # image_id = int((total_xml[i]).split('.')[0])
    # if image_id == 361 or image_id == 362 or image_id == 355:  # remove error pictures
    #     print(total_xml[i])
    #     continue

    name = total_xml[i][:-4] + '\n'
    if i in trainval:
        ftrainval.write(name)
        if i in train:
            ftest.write(name)
        else:
            fval.write(name)
    else:
        ftrain.write(name)

ftrainval.close()
ftrain.close()
fval.close()
ftest.close()
