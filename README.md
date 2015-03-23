Stereo rectification and stereo matching for perspective and fisheye (omnidirectional) camera, based on Opencv. 

The feature of this project is that **two rectification methods** are available. 

The **first** is conventional perspective rectification, i.e., the rectified images are perspective so that stereo matching can be applied in one line. 
However, since the fields of view (FOV) for omnidirectional camera is very large, conventional rectified image should be larger to contain all scene view. The **second** one is a longitude-latitude rectification. It is based on sphere model of omnidirectional camera. Each pixel on rectified image is one point on unit sphere, and the location on image represents the longitude and latitude on sphere. It means that all points on unit have longitude and latitude and then have locations on rectified images. The benefit is that unlike perspective rectification, all details of the origional image can be preserved and the black region is not large on boundary. In addition, stereo matching is also avaible in a line because a horizontal line in 3D space is projected on a latitude line on unit sepher.

See Folder 'Images' to see the origional images and rectified images using the above two methods.

The parameters of camera are parsed from an ini file. There is a class to read parameters in Folder 'Utilities'. The format of ini file must be strictly followed by the example file named by 'EXAMPLE.ini'.

Methods for rectification is in Folder 'Stereo'.

The interfaces are about the same with rectification functions in Opencv.

