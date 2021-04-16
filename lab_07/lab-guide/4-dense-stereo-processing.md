# Step 4: Dense stereo processing

## 6. Dense stereo processing
Go to [lab_7.cpp:92](https://github.com/tek5030/lab_07/blob/main/lab_7.cpp#L92).
Here, an empty `cv::Ptr<cv::stereo::StereoMatcher>` is created.
Your task is to create a proper [`cv::stereo::StereoBinarySGBM` matcher](https://docs.opencv.org/4.0.1/d1/d9f/classcv_1_1stereo_1_1StereoBinarySGBM.html) instead.

## 7. Dense depth instead from dense disparity
Go to [lab_7.cpp:99](https://github.com/tek5030/lab_07/blob/main/lab_7.cpp#L99).
Here, `dense_depth` is set equal to `dense_disparity`.
Your task is to compute the correct `dense_depth` in meters.

Hint: No for-loop required!

## Experiments
- Activate the projector. 
  Does it benefit the sparse processing or the dense processing or both?
- Take a look at [the examples in camera-library](https://github.com/tek5030/camera-library/tree/main/example).
  Notice that you can capture depth directly from the camera!
  Play around with these examples.
- Run `realsense-viewer` from the console.
  This is Intel's interface to the camera.
  It lets you capture images, and even visualize the results in 3D!
  
