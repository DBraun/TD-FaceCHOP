[//]: # (For development of this README.md, use http://markdownlivepreview.com/)

# TD-FaceCHOP
#### Face landmark detection with OpenCV and dlib (in TouchDesigner).

![](docs/images/header.png)

FaceCHOP takes an image and an expected camera horizontal field-of-view. The output is 4 channels and many samples. Each detected face corresponds to 71 samples. The first face involves samples 0-70; the second is 71-141; then 142-212 and so on. Of the 71 samples, the first 68 samples are the 2D (tx/ty) locations of the face landmarks. These correspond to the indices of the [iBUG 300-W dataset](https://ibug.doc.ic.ac.uk/resources/facial-point-annotations/).

After these 68 samples, the next sample is a rectangular box indicating where the face is. Basically, it's (center x, center y, size x, size y). Check the example TouchDesigner project for more context.
The next sample is the translation of the face (tx, ty, tz). The fourth channel here is unused.
The next sample is the rotation of the face (rx, ry, rz). The fourth channel here is unused.

FaceCHOP has a toggle for using face landmark detection, so if you only care about getting the rectangular box of a face, leave it unchecked. Note that if you use face landmark detection, which is a step in finding the pose (rotation & translation) of a face, then you need to follow the licensing of iBUG. iBUG does not allow commercial use, so contact them for more information.

There's a useful parameter "Facerectframeskip" whose label is "Face Rectangle Frame Skip." Processing the rectangles of faces is far more time-consuming that placing the landmarks on the faces. If you want to get some speed savings and can sacrifice a little accuracy of the landmarks, it can be a good idea to skip 1, maybe 2 frames. If the value is 0, then you'll calculate new face rectangles for every frame.

# Installation
In order to run the demo project, there are three basic requirements:
* Download `TD-FaceCHOP.dll` from the [Releases](https://github.com/DBraun/TD-FaceCHOP/releases) page and place it in the `Plugins` folder in this repo.
* Download [OpenCV 4.1.1](https://opencv.org/releases/) to `C:/tools/opencv`. Copy the file `C:\tools\opencv\build\x64\vc15\bin\opencv_world411.dll` into `TD-FaceCHOP\Plugins`. Wherever you use `TD-FaceCHOP.dll`, `opencv_world411.dll` must be its direct neighbor.
* Download the dlib pretrained [shape_predictor_68_face_landmarks.dat](http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2). **By using this file you must obey the licensing of the iBUG dataset.** Use an application such as 7-zip to turn the `.bz2` file into a `.dat` file and place it in the root of this repo, sibling to `FaceCHOP.toe`. You can use a custom parameter on Face-CHOP to keep this model in a different location. The model is also available [here](https://github.com/davisking/dlib-models).

That's it! The remaining instructions in this guide are for compiling `TD-FaceCHOP.dll` in case you want to use a different version of OpenCV or a different landmarks model.

## OpenCV
Download [OpenCV](https://opencv.org/releases/). I use 4.1.1, but you can probably use other versions.
On Windows, I've placed it at `C:/tools/opencv` so that I have `C:\tools\opencv\build\x64\vc15\lib\opencv_world411.lib`.

## dlib
Clone [dlib](https://github.com/davisking/dlib) to `C:\tools\dlib`. These are my three steps for building on Windows. Open a cmd window into `C:\tools\dlib`. Then

    mkdir build; cd build;
    cmake -g "Visual Studio 16 2019 Win64" -T host=x64 -DCMAKE_INSTALL_PREFIX=install .. -DUSE_AVX_INSTRUCTIONS=1;
    cmake --build . --config Release --target INSTALL

You should end up with `C:\tools\dlib\build\install\lib\dlib19.17.99_release_64bit_msvc1922.lib`

## Visual Studio Properties
If you install OpenCV or dlib with other methods or in other locations, you'll need to modify the properties of the Visual Studio solution. For my build setup, 
note how both the OpenCV and dlib directories have been added to "Additional Include Directories"
![](docs/images/properties_1.png)
Note how both the OpenCV and dlib directories have been added to "Additional Library Directories"
![](docs/images/properties_2.png)
**Note how both the OpenCV and dlib `.lib` files have been added to "Additional Dependencies"**
![](docs/images/properties_3.png)

# To do
* Mac OS support
* Avoid using OpenCV Mat container
* Try other methods
 * [https://github.com/1adrianb/face-alignment](https://github.com/1adrianb/face-alignment)
 * [https://github.com/ageitgey/face_recognition/](https://github.com/ageitgey/face_recognition/)
 * [https://github.com/cleardusk/3DDFA](https://github.com/cleardusk/3DDFA) for a better 3D mesh
 * [https://github.com/yinguobing/head-pose-estimation](https://github.com/yinguobing/head-pose-estimation)

# Thanks
* [http://dlib.net/webcam\_face\_pose\_ex.cpp.html](http://dlib.net/webcam\_face\_pose\_ex.cpp.html)
* [https://www.learnopencv.com/head-pose-estimation-using-opencv-and-dlib/](https://www.learnopencv.com/head-pose-estimation-using-opencv-and-dlib/)
* [http://aifi.isr.uc.pt/HeadPoseEstimation.html](http://aifi.isr.uc.pt/HeadPoseEstimation.html)
* [https://github.com/lincolnhard/head-pose-estimation](https://github.com/lincolnhard/head-pose-estimation)
* [https://github.com/mourendxu/TD-OpenCV3TOP](https://github.com/mourendxu/TD-OpenCV3TOP)

# Dependent Licenses
[iBUG 300-W dataset](https://ibug.doc.ic.ac.uk/resources/facial-point-annotations/) does not offer commercial usage. This dataset was used to make `shape_predictor_68_face_landmarks.dat`, which is what makes it possible to identify the 68 landmarks on a face.

FaceCHOP also uses numbers from [http://aifi.isr.uc.pt/HeadPoseEstimation.html](http://aifi.isr.uc.pt/HeadPoseEstimation.html) that represent an expected 3D head shape in centimeters. That project is copyright of [Pedro Martins](pedromartins@isr.uc.pt) but licensed under GNU General Public License.
