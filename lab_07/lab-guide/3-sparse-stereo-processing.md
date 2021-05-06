# Step 3: Sparse stereo processing
Now, let's do some stereo processing!

## 1. Improve point correspondences
Go to `SparseStereoMatcher::extractGoodMatches()` in [sparse_stereo_matcher.cpp:36](https://github.com/tek5030/lab_07/blob/main/sparse_stereo_matcher.cpp#L36).
Your task is to exploit what we know about the epipolar geometry to remove bad matches.
We should also remove matches that result in impossible disparities.

Hint: Use the fact that the images are rectified, and now follow the model of ideal stereo geometry.

## 2. Compute disparity
Go to `SparseStereoMatcher::computeDisparities()` in [sparse_stereo_matcher.cpp](https://github.com/tek5030/lab_07/blob/main/sparse_stereo_matcher.cpp).
This function currently sets all disparities to 0.
Your task is to make it return the proper disparities instead.

Hint: It's in lecture 7.2.

## 3. Compute depth
Go to [lab_7.cpp:74](https://github.com/tek5030/lab_07/blob/main/lab_7.cpp#L74).
We will here compute the depth in meters for each correspondence.
Currently the depth is set to be equal to the disparity.
Your task is to compute the proper depth.

Hint: Look at ```SparseStereoMatcher::computeDisparites``` to see the structure of each ```cv::Vec3d``` in 
```stereo_matcher::point_disparities()```.

## 4. Compute 3D points
Go to [lab_7.cpp:85](https://github.com/tek5030/lab_07/blob/main/lab_7.cpp#L85).
Here, `std::vector<cv::Vec3d> world_points` is created, but it is empty.
Your task is to compute the 3D point corresponding to each point correspondence.

Hint: point_disparities and world_points are related by the homography **Q** (see the lectures).
So use `cv::perspectiveTransform()`!

## 5. Add intensity to the point cloud
Go to [lab_7.cpp:80](https://github.com/tek5030/lab_07/blob/main/lab_7.cpp#L80).
Here, the `point_colors` are currently set to 255 (white).
Your task is to replace it with the intensity level of the corresponding pixel in the left image.

Please continue to the [next step](4-dense-stereo-processing.md), where we explore dense stereo processing!
