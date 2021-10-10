# Social-Distancing-TEK5030

Created a program monitoring whether the Covid-19 social distancing rules were upheld, tested on a stereo-camera dataset. We used a pre-trained Yolov3 network to detect people and a dense stereo estimator to find their positions in 3D space. The shortest distance each person had to another person were calculated. The end result can be seen in the image below, it can also be seen as a gif (socialdistance.gif).

![socialdist](https://user-images.githubusercontent.com/36857118/129191358-a19b831f-aedc-459d-8073-40d32ca8c7bf.PNG)
## Installing dependecies
All dependencies can be installed through https://github.com/tek5030/setup_scripts
## How to run
Run these commands to run the person placer
```
cd person_placer
make
./person_placer
```

