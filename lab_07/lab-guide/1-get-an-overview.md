# Step 1: Get an overview
We will start by presenting an overview of the lab and the contents of this project.

## Stereo camera
In this lab we will experiment with stereo processing using the [Kitti Dataset](http://www.cvlibs.net/datasets/kitti/index.php) which will provide to us:
  - Raw (unsynced+unrectified) and processed (synced+rectified) grayscale stereo sequences (0.5 Megapixels, stored in png format)
  - Raw (unsynced+unrectified) and processed (synced+rectified) color stereo sequences (0.5 Megapixels, stored in png format)
  - Calibration data (stored as text file)

We will use the unrectified grayscale images from the dataset, and perform rectification using the provided calibration data.

For the course, we have implemented a simple `KittiCamera` in the [camera-library](https://github.com/tek5030/camera-library) repository.
It can be used to load image pairs and calibration data from the downloaded datasets.

If you have two webcameras available, we have also implemented the [dual_camera](https://github.com/tek5030/camera-library/blob/main/include/tek5030/dual_camera.h) interface to `cv::VideoCapture`, which lets you use two ordinary cameras to capture stereo images.
You will in this case need to change the lab code to use this interface instead.
Additionally, you'll get some practical experience with stereo calibration because we must then start by estimating the calibration parameters ourselves.
Take a look at the [examples in camera-library](https://github.com/tek5030/camera-library/tree/main/example) to see how to use the camera interfaces.

Please install [camera-library](https://github.com/tek5030/camera-library) now by writing these commands in you terminal (from an appropriate location):
```bash
git clone --depth 1 https://github.com/tek5030/camera-library
cmake -S camera-library -B camera-library/build
sudo cmake --build camera-library/build --target install
# If you want to remove the code:
sudo rm -rf camera-library
```

### Download the Kitti data
Let's start downloading the data right away!
If you want to construct your own stereo camera, you may of course skip this step.

Go to [http://www.cvlibs.net/datasets/kitti/raw_data.php](http://www.cvlibs.net/datasets/kitti/raw_data.php?type=campus)
to find the different datasets. Select one of the categories and download a dataset. You will see

> **Downloads:** `[unsynced+unrectified data] [synced+rectified data] [calibration]`

- Download `[unsynced+unrectified data]` and extract the files to some directory on your computer.
  - If you are short on disk space, you may delete the `oxts` and `velodyne_points` right away.
- Download `[calibration]` and extract the files to some directory on your computer.


The exctracted data should look like this
```
Downloads
.
├── 2011_09_28
│   └── 2011_09_28_drive_0016_extract
│       ├── image_00
│       │   ├── data
│       │   └── timestamps.txt
│       ├── image_01
│       │   ├── data
│       │   └── timestamps.txt
│       ├── image_02
│       │   ├── data
│       │   └── timestamps.txt
│       ├── image_03
│       │   ├── data
│       │   └── timestamps.txt
│       ├── oxts
│       │   ├── data
│       │   ├── dataformat.txt
│       │   └── timestamps.txt
│       └── velodyne_points
│           ├── data
│           ├── timestamps_end.txt
│           ├── timestamps_start.txt
│           └── timestamps.txt
└── 2011_09_28_calib
    └── 2011_09_28
        ├── calib_cam_to_cam.txt
        ├── calib_imu_to_velo.txt
        └── calib_velo_to_cam.txt
```

In the lab, you must specify the directories containing data and calibration.
- The data directory is the one containing the `image_xx` folders, e.g. `/path/to/download/2011_09_28/2011_09_28_drive_0016_extract/`
- The calib directory is the one containing the `calib_cam_to_cam.txt` file, e.g. `/path/to/download/2011_09_28_calib/2011_09_28/`


## Lab overview
The main steps of today's lab are:

- Stereo calibration (please read through this part even if you are using the kitti dataset)
  - We will perform calibration by running the project in the [stereo_calibration](https://github.com/tek5030/stereo_calibration) repository.

- Sparse stereo processing
  - Establish point correspondences based on keypoint descriptors
  - Detect and discard bad correspondences based on the epipolar geometry
  - For each good correspondence:
    - Determine the disparity
    - Compute the depth in meters
    - Compute the 3D point

- Dense stereo processing
   - Experiment with existing implementations to produce dense depth images

## Introduction to the project source files

We have chosen to distribute the code on the following files:
- *main.cpp*
  
  Starts lab 7, catches any exceptions and prints their error message on the console.

- *lab_7.h*, *lab_7.cpp*

  Implements the lab 7 loop. 
  You will add missing functionality to this loop.
  
- *stereo_calibration.h*, *stereo_calibration.cpp*

  Contains the definition of a class `StereoCalibration` that contains the results of our stereo calibration. 
  We will use this to rectify raw images from two cameras into properly aligned stereo images.
  You can also choose to read the calibration parameters directly from the RealSense camera, when capturing rectified images.

- *sparse_stereo_matcher.h*, *sparse_stereo_matcher.cpp*

  Contains the definition of a class `SparseStereoMatcher`. 
  This class is able to extract and match keypoints from both images in a `StereoPair`. 
  We will use these keypoint correspondences to do sparse stereo processing.
  
  You will add missing functionality to this class. 

- *viewer_3d.h*, *viewer_3d.cpp*

  Helper class that displays the processed stereo result as a 3D point cloud.
  
- *visualization.h*, *visualization.cpp*

  Helper class for visualizing matched point correspondences and estimated depth to image points.

- *cv_stereo_matcher_wrap.h*, *cv_stereo_matcher_wrap.cpp*

  Helper class for dense stereo processing.
  
In addition, we will use the following repositories:
  - [camera-library](https://github.com/tek5030/camera-library)
  - [stereo_calibration](https://github.com/tek5030/stereo_calibration)
    
Please continue to the [next step](2-stereo-calibration.md) to get started!

Good luck!
