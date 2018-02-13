# point-cloud-visualizer
Tutorial to demonstrate Point Cloud visualization

## Setps to compile the project
1. Make sure to download compelte repository. Use `git clone` or download directory as per convenience
1. Now go to the project `cd point-cloud-visualization`
1. In order to compile the project, make `build` direcoty first. Use following command `mkdir build`
1. Go inside the build directory. Use following command `cd build`
1. Run `cmake` tool. Use following command `cmake ..`
1. Run `make` tool. Use following command `make`
After following the above instructions, you should be able to see two executable files inside build directory.

## Steps to run the project
Make sure to follow the setps to compile the project. The project contains following three files:
1. **simple_visualizer**: Go inside `build` directory and run it directly i.e., `./simple_visualizer`
1. **advance_visualizer**: Go inside `build` directory and run it directly i.e., `./advance_visualizer`
1. **pick_point**: Go inside `build` directory and run it by providing PCD file as input i.e., `./pick_point capture.pcd`

## Tips
1. Press `h` key in order to see more controls, such as take screenshot etc.
1. To pick a point in cloud, hold down `SHIFT` key while left-clicking. 
1. In order to change the camera angle, camera pose can also be provided as a part of command line argument as following:
`./advance_visualizer -cam 0.026123,26.123/-0.00599269,0.00252074,1.00064/0.160405,0.457474,-1.54746/0.0138908,-0.984493,-0.17487/0.8575/1863,1176/57,24`
