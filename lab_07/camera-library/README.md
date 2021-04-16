## tek5030 camera library
This library contains wrappers for cameras used in the course tek5030.
The classes are designed in such a way that it should be as easy as possible to switch between cameras.

The following camera classes are available
- `tek5030::OpenCVCamera`, _super thin wrapper around cv::VideoCapture_
- `tek5030::DualCamera`, _capture images simultaneously-ish from two cv::VideoCapture objects so that we can use ordinary webcams as stereo cameras_
- `tek5030::KittiCamera`, _capture image pairs from a [Kitti Dataset](http://www.cvlibs.net/datasets/kitti/raw_data.php), downloaded by you._

### Dependencies
- OpenCV

### Install this library
```bash
cd <path to library>
mkdir build && cd $_
cmake .. [-DCMAKE_INSTALL_PREFIX=<custom prefix>]
make
(sudo) make install
```
### Use this library in your project
In your _CMakeLists.txt_:
```cmake
find_package(tek5030 CONFIG REQUIRED)
add_executable(my_exe my_main.cpp)
target_link_libraries(my_exe PRIVATE tek5030::tek5030)
```

Stream single frames:
```c++
#include "tek5030/opencv_camera.h"     // Super thin wrapper around cv::VideoCapture
#include "tek5030/dual_camera.h"       // Stereo from two cv::VideoCapture cameras

using namespace tek5030;

// Initialize with device ID
OpenCVCamera cam(0);

// Initialize with device IDs 
DualCamera cam(0,1);

// Initialize with path to dataset and calibration. Select color or monochrome camera pair.
bool color = false;
KittiCamera cam("path_to_data", "path_to_calib", color)
```

Fetch a StereoPair:
```c++
// Available from DualCamera or KittiCamera
tek5030::StereoPair stereo_pair = cam.getStereoPair();
// or
auto [left, right] = cam.getStereoPair();
```
Access and configure the internal `cv::VideoCapture` object:
```c++
// Available from DualCamera or OpenCVCamera
auto& cap = cam.getVideoCapture();
cap.set(cv::CAP_PROP_xyz, value); 
```
Retreive the calibration data for a `KittiCamera`
```cpp
const auto calibration_data = cam.getCalibration(KittiCamera::Cam::GrayLeft);
```

