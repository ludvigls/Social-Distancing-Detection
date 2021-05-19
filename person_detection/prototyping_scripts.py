from person_detection import BboxDetector
from PIL import Image
from os import listdir
from os.path import join
from tqdm import tqdm
import json
import matplotlib.pyplot as plt
DATASET= "lab_07/2011_09_28_drive_0045_extract/2011_09_28/2011_09_28_drive_0045_extract/image_03/data"
def compute_dataset_bbox():
    """

    :return: (saved) dictionary on form {filename: list of bbox centroids}

    """
    detector = BboxDetector("DeepModels/yolov3.weights", use_cuda=False)
    data_dict = {}
    for i, name in tqdm(enumerate(listdir(DATASET))):
        image = Image.open(join(DATASET,name))
        centres = detector.get_centroids(image)
        data_dict[name] = centres.tolist()
        # detector.draw_bbox(image)
        # plt.imshow(image)
        # plt.scatter(centres[:,0], centres[:,1])
        # plt.show()
    print(data_dict)
    json.dump(data_dict, open("new_dataset_image_03_detections.json", "w"))



if __name__ == '__main__':
    compute_dataset_bbox()
