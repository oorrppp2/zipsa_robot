#!/home/robot/anaconda3/envs/pytorch_env/bin/python
import os
import sys
import rospy
from cv_bridge import CvBridge
import message_filters
from skimage import io, transform
import torch
import torchvision
from torch.autograd import Variable
from torchvision import transforms
import torch.nn.functional as F
import torch.nn as nn
import time

import numpy as np
from PIL import Image as PIL_Image
import glob
import cv2

""" U2-Net """
from model.data_loader import RescaleT
from model.data_loader import ToTensorLab
from model import U2NET # full size version 173.6 MB

""" HSNet """
from model.hsnet import HypercorrSqueezeNetwork

from vision_msgs.msg import SegResult
from std_msgs.msg import Empty, String, Bool, Header, Float64, Int8
from sensor_msgs.msg import CompressedImage, Image, JointState
from sensor_msgs.msg import PointCloud2, PointField

root_dir = rospy.get_param("root_dir", default="")
print("...load U2NET---173.6 MB")
net = U2NET(3,1)
net.load_state_dict(torch.load(root_dir+"/src/model/u2net.pth"))
net.eval()

HSNet = HypercorrSqueezeNetwork('resnet101', False)
HSNet.eval()
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
# Logger.info('# available GPUs: %d' % torch.cuda.device_count())
HSNet = nn.DataParallel(HSNet)
HSNet.to(device)
# HSNet.load_state_dict(torch.load(root_dir+"/src/model/hsnet.pt"))
HSNet.load_state_dict(torch.load(root_dir+"/src/model/best_model.pt"))


class SegModule:    # U2Net + HSNet
    def __init__(self):

        self.segmentation_result_pub = rospy.Publisher("/segmentation_result", SegResult, queue_size=1)
        self.segmentation_result_pub_Image = rospy.Publisher("/segmentation_result_Image", Image, queue_size=1)

        self.br = CvBridge()
        self.image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.RGBD_sub_callback)


        self.target_id_sub = rospy.Subscriber('/target_id', Int8, self.target_id_callback)
        self.target_id = -1
        # self.target_id = 11


        # print("...load U2NET---173.6 MB")
        # self.net = U2NET(3,1)
        # self.net.load_state_dict(torch.load(self.root_dir+"/src/model/u2net.pth"))
        # self.net.eval()

    def target_id_callback(self, msg):
        self.target_id = msg.data
        print("target id : ", msg.data)

    # normalize the predicted SOD probability map
    def normPRED(self, d):
        ma = torch.max(d)
        mi = torch.min(d)

        dn = (d-mi)/(ma-mi)

        return dn


    def RGBD_sub_callback(self, image_msg, depth_msg):
        if self.target_id > 0:
            image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1).copy()
            depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width, -1).copy()
            img = image.copy()
            ori_img = image.copy()

            r, g, b = cv2.split(img)
            img = cv2.merge((b, g, r))

            # U2Net
            transform=transforms.Compose([RescaleT(320), ToTensorLab(flag=0)])

            label = np.zeros(image.shape)
            imidx = np.array([self.target_id])
            sample = {'imidx':imidx, 'image':image, 'label':label}
            sample = transform(sample)

            inputs_test = sample['image'].unsqueeze(dim=0)
            inputs_test = inputs_test.type(torch.FloatTensor)

            u2_net_time = time.time()
            d1,d2,d3,d4,d5,d6,d7= net(inputs_test)
            rospy.loginfo('U2-Net detection time: {0}sec'.format(time.time() - u2_net_time))

            # # normalization
            pred = d1[:,0,:,:]
            pred = self.normPRED(pred)

            predict = pred
            predict = predict.squeeze()
            predict_np = predict.cpu().data.numpy()

            imo = cv2.resize(predict_np * 255, (image.shape[1],image.shape[0]), interpolation=cv2.INTER_LINEAR)
            salient_detection_img = ( np.array(imo) / 255.0 ).astype(np.float32)

            image[salient_detection_img < 0.5] = 0
            r, g, b = cv2.split(image)
            segmented_image = cv2.merge((b, g, r))

            del d1,d2,d3,d4,d5,d6,d7



            # HSNet
            img_size=400
            img_mean = [0.485, 0.456, 0.406]
            img_std = [0.229, 0.224, 0.225]

            transform = transforms.Compose([transforms.Resize(size=(img_size, img_size)),
                                                transforms.ToTensor(),
                                                transforms.Normalize(img_mean, img_std)])

            # query_name = image.copy()
            support_image_names = []
            support_label_names = []
            for i in range(5):
                support_image_names.append(root_dir+"/support_sets/{0}/color_{1}.png".format(self.target_id, str(i)))
                support_label_names.append(root_dir+"/support_sets/{0}/label_{1}.png".format(self.target_id, str(i)))

            def read_mask(img_name):
                r"""Return segmentation mask in PIL Image"""
                mask = torch.tensor(np.array(PIL_Image.open(img_name)))[:,:,0]
                return mask

            def read_img(img_name):
                r"""Return RGB image in PIL Image"""
                return PIL_Image.open(img_name)

            image_ori = segmented_image.copy()
            query_img = PIL_Image.fromarray(segmented_image.copy())
            support_imgs = [read_img(name) for name in support_image_names]
            support_cmasks = [read_mask(name) for name in support_label_names]

            org_qry_imsize = query_img.size


            query_img = transform(query_img)
            support_imgs = torch.stack([transform(support_img) for support_img in support_imgs])

            support_masks = []
            support_ignore_idxs = []
            for scmask in support_cmasks:
                scmask = F.interpolate(scmask.unsqueeze(0).unsqueeze(0).float(), support_imgs.size()[-2:], mode='nearest').squeeze()
                support_masks.append(scmask)
            support_masks = torch.stack(support_masks)

            def to_cuda(batch):
                for key, value in batch.items():
                    if isinstance(value, torch.Tensor):
                        batch[key] = value.cuda()
                return batch

            batch = {'query_img': query_img.unsqueeze(0),
                        'org_query_imsize': org_qry_imsize,

                        'support_imgs': support_imgs.unsqueeze(0),
                        'support_masks': support_masks.unsqueeze(0),
                        'class_id': torch.IntTensor([4])}

            batch = to_cuda(batch)
            HSNet_time = time.time()
            pred_mask = HSNet.module.predict_mask_nshot(batch, nshot=len(support_image_names))
            rospy.loginfo('HSNet detection time: {0}sec'.format(time.time() - HSNet_time))

            longerside = max(org_qry_imsize[0], org_qry_imsize[1])
            backmask = torch.ones(1, longerside, longerside).cuda()*255
            target = backmask.clone().long()

            output = F.interpolate(pred_mask.unsqueeze(0), size=image_ori.shape[:2], mode='bilinear', align_corners=True)

            output = output[0][0]

            output_cpu = output.cpu().data.numpy()
            output_cpu = output_cpu[:480,:]
            color = np.asarray([0.,0.,255.], dtype=np.float)
            masked_image = image_ori.copy().astype(np.float)

            masked_image[output_cpu > 0] *= 0.5
            masked_image[output_cpu > 0] += color * 0.5
            masked_image = masked_image.astype(np.uint8)

            other_region = imo.astype(np.uint8)
            target_region = output_cpu.astype(np.uint8)
            other_region_msg = self.br.cv2_to_imgmsg(other_region)
            target_region_msg = self.br.cv2_to_imgmsg(target_region)
            target_region_msg.header.stamp = rospy.Time.now()

            seg_message = SegResult()
            seg_message.frame_id = self.target_id
            # seg_message.frame_id = cfg.dataset.class_names[classes[selected_class]]
            seg_message.roi = []
            seg_message.target_region = target_region_msg
            seg_message.other_region = other_region_msg
            seg_message.color = image_msg
            seg_message.depth = depth_msg
            self.segmentation_result_pub.publish(seg_message)
            self.segmentation_result_pub_Image.publish(target_region_msg)

            # cv2.imshow("Input image", ori_img)
            # cv2.imshow("U2-Net result", imo)
            # cv2.imshow("U2-Net result", salient_detection_img)
            # cv2.waitKey(1)
            # cv2.imshow("masked_image", masked_image)

            trial = 1
            cv2.imwrite("/home/robot/IROS_Experiment/exp_video/{0}/image_{1}.png".format(str(self.target_id), str(trial)), img)
            cv2.imwrite("/home/robot/IROS_Experiment/exp_video/{0}/depth_{1}.png".format(str(self.target_id), str(trial)), depth)
            cv2.imwrite("/home/robot/IROS_Experiment/exp_video/{0}/masked_image_{1}.png".format(str(self.target_id), str(trial)), masked_image)
            cv2.imwrite("/home/robot/IROS_Experiment/exp_video/{0}/segmented_{1}.png".format(str(self.target_id), str(trial)), segmented_image)

            # cv2.imshow("output", output_cpu.astype(np.float32))
            # cv2.waitKey(1)

            self.target_id = -1

        # else:
        #     image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1).copy()
        #     depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width, -1).copy()
        #     print(depth[depth > 0])

        #     cv2.imshow("image", image)
        #     cv2.imshow("depth", depth)
        #     cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('SegModule', anonymous=False)
    n = SegModule()

    rospy.spin()
