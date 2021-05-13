import torch
import torch.nn as nn
from Pose2Seg.modeling.build_model import Pose2Seg
from os.path import join
from torchvision import transforms,datasets
from torch.autograd import Variable
from models import Darknet
import utils.utils as utils
from mmpose.models.detectors import BottomUp

PATH_TO_MODEL_DICT = "DeepModels/"
config_path='config/yolov3.cfg'
weights_path='config/yolov3.weights'
class_path='config/coco.names'



class PoseSkeletonModel(nn.Module):
    """
    Bottom-up pose estimation
    """
    def __init__(self, weights):
        super(PoseSkeletonModel, self).__init__()
        self.pretrained = None #BottomUp()

    def forward(self, x):
        return self.pretrained.forward(x)


class InstanceSegmentationModel(nn.Module):
    """
        Deep-learning model that performs instance segmentation of persons;
        robust to occlusion due to pose-estimation based approach.
        See https://arxiv.org/abs/1803.10683 for details

        TODO Pose is estimated using https://github.com/open-mmlab/mmpose

    """
    def __init__(self, pose_weights, pose2seg_weights):
        super(InstanceSegmentationModel, self).__init__()
        self.img2pose = PoseSkeletonModel(join(PATH_TO_MODEL_DICT, pose_weights))
        self.pose2seg = Pose2Seg().init(join(PATH_TO_MODEL_DICT, pose2seg_weights))

    def forward(self, image):
        keypoints = self.img2pose.forward(image)
        segmasks = self.pose2seg.forward(image, keypoints)
        return keypoints, segmasks


class ObjectDetectionModel(nn.Module):
    """
        YOLOv3 based detection model

        TODO https://towardsdatascience.com/object-detection-and-tracking-in-pytorch-b3cf1a696a98

    """
    def __init__(self, state_dict_path, img_size):
        super(ObjectDetectionModel, self).__init__()
        self.model = Darknet(config_path, img_size=img_size)
        self.model.load_weights(state_dict_path)
        classes = utils.load_classes(class_path)

    def forward(self, img):
        # TODO adjust such that only people are detected
        return self.model.forward(img)

if __name__ == '__main__':
    print("Testing...")