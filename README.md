# Two-view-measurement
Two-view measurement based on SFM

1 Installation:
1) Install MCRInstaller.exe version 9.4 (R2018a)(http://www.mathworks.com/products/compiler/mcr/index.html)
2) New a environment variable in your computer and add path to the installed compiler into that variable

2 Steps for running the application:
1) Check the sensor size and focal length of your phone (https://www.devicespecifications.com/en) 
Enter the sensor(ccd) width (which is the longer side of the sensor size of your phone).
2) Select a video
2) Enter the reference distance got from AR [app](https://github.com/qianzhang1212/ARCoreMeasure)
3) Select two points as target

3 Notes
1 To get good performance, walk a short distance (~40cm, e.g. a small step) to guarantee the first and last frame have enough overlap
2 When you hold youe phone horizontally, check "Rotate" before you select video, if not, leave "Rotate" unchecked
