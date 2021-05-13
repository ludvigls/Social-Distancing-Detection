from person_detection_models import *
import matplotlib.pyplot as plt
from PIL import Image
import torch
import numpy as np
import random
import matplotlib.patches as patches
classes = utils.load_classes("config/coco.names")

conf_thres=0.5
nms_thres=0.4

class BboxDetector:
    def __init__(self, path, img_size=416, use_cuda=True):
        self.model = ObjectDetectionModel(path, img_size)
        self.img_size = img_size
        self.model.eval()
        if use_cuda:
            self.model.cuda()

    def get_bbox(self, img):
        """
            FORMAT: x1, y1, x2, y2, conf, cls_conf, cls_pred
        :return:
        """
        ratio = min(self.img_size / img.size[0], self.img_size / img.size[1])
        imw = round(img.size[0] * ratio)
        imh = round(img.size[1] * ratio)
        img_transforms = transforms.Compose([transforms.Resize((imh, imw)),
                                             transforms.Pad((max(int((imh - imw) / 2), 0),
                                                             max(int((imw - imh) / 2), 0),
                                                             max(int((imh - imw) / 2), 0),
                                                             max(int((imw - imh) / 2), 0)), (128, 128, 128)),
                                             transforms.ToTensor(),
                                             ])
        image_tensor = img_transforms(img).float()
        image_tensor = image_tensor.unsqueeze_(0)
        # run image through model
        with torch.no_grad():
            detections = self.model(image_tensor)
            detections = utils.non_max_suppression(detections, 80, conf_thres, nms_thres)
        #NOTE there is room for improvement here; we perform computations for all classes, and only select persons after the fact
        if detections[0] is None:
            return None
        mask = detections[0][:, -1] == 0
        return detections[0][torch.nonzero(mask)].permute(0,2,1).squeeze()

    def get_centroids(self, img):
        try:
            detections = self.get_bbox(img)
            img = np.array(img)
            pad_x = max(img.shape[0] - img .shape[1], 0) * (self.img_size / max(img.shape))
            pad_y = max(img.shape[1] - img.shape[0], 0) * (self.img_size / max(img.shape))
            unpad_h = self.img_size - pad_y
            unpad_w = self.img_size - pad_x
            coords = []
            for x1, y1, x2, y2, conf, cls_conf, cls_pred in detections:
                box_h = int(((y2 - y1) / unpad_h) * img.shape[0])
                box_w = int(((x2 - x1) / unpad_w) * img.shape[1])
                y1 = int(((y1 - pad_y // 2) / unpad_h) * img.shape[0])
                x1 = int(((x1 - pad_x // 2) / unpad_w) * img.shape[1])
                coords.append((x1+box_w//2, y1+box_h//2))
            return np.array(coords)
        except:
            return np.array(0) #fixes a bug that has no reason to exist


    def draw_bbox(self, img):
        """
            Stolen from a tutorial. #TODO effectivize
        """
        detections = self.get_bbox(img)
        img = np.array(img)
        plt.figure()
        fig, ax = plt.subplots(1, figsize=(12, 9))
        ax.imshow(img)

        cmap = plt.get_cmap('tab20b')
        colors = [cmap(i) for i in np.linspace(0, 1, 20)]

        pad_x = max(img.shape[0] - img.shape[1], 0) * (self.img_size / max(img.shape))
        pad_y = max(img.shape[1] - img.shape[0], 0) * (self.img_size / max(img.shape))
        unpad_h = self.img_size - pad_y
        unpad_w = self.img_size - pad_x

        if detections is not None:
            unique_labels = detections[:, -1].cpu().unique()
            n_cls_preds = len(unique_labels)
            bbox_colors = random.sample(colors, n_cls_preds)
            # browse detections and draw bounding boxes
            for x1, y1, x2, y2, conf, cls_conf, cls_pred in detections:
                box_h = ((y2 - y1) / unpad_h) * img.shape[0]
                box_w = ((x2 - x1) / unpad_w) * img.shape[1]
                y1 = ((y1 - pad_y // 2) / unpad_h) * img.shape[0]
                x1 = ((x1 - pad_x // 2) / unpad_w) * img.shape[1]
                color = bbox_colors[int(np.where(
                     unique_labels == int(cls_pred))[0])]
                bbox = patches.Rectangle((x1, y1), box_w, box_h,
                     linewidth=2, edgecolor=color, facecolor='none')
                ax.add_patch(bbox)
                plt.text(x1, y1, s=classes[int(cls_pred)],
                        color='white', verticalalignment='top',
                        bbox={'color': color, 'pad': 0})
        plt.axis('off')
        # save image
        plt.show()

class PoseDetector:
    def __init__(self):
        pass

    def get_pose(self, img):
        pass

    def draw_pose(self, img):
        pass


class InstanceSegmentation:
    def __init__(self):
        pass

    def get_segmask(self, img):
        pass

    def draw_segmask(self, img):
        pass


if __name__ == '__main__':
    PATH_TO_TEST_IMAGE = "Pose2Seg/data/OCHuman/images/000092.jpg"
    test_image = Image.open(PATH_TO_TEST_IMAGE)
    bbox = BboxDetector("DeepModels/yolov3.weights", use_cuda=False)
    # bbox.get_bbox(test_image)
    bbox.draw_bbox(test_image)
    plt.imshow(test_image)
    plt.show()
    print("Testing bounding-box based detection...")

