# point-cloud-visualizer
Tutorial to demonstrate visualization of Point Clouds in PCL

## Compilation
```console
$ git clone https://github.com/ravijo/point-cloud-visualization.git
$ cd point-cloud-visualization
$ mkdir build
$ cd build
$ cmake ..
$ make
```
After following the above instructions, you should be able to see two executable files inside the build directory.

## Project Structure
The project contains the following files:
1. simple_visualizer.cpp
    ```console
    $ cd build
    $ ./simple_visualizer
    ```
2. advance_visualizer.cpp
    ```console
    $ cd build
    $ ./advance_visualizer
    ```
3. pick_point.cpp
    ```console
    $ cd build
    $ ./pick_point capture.pcd
    ```
4. surface_normal_visualizer.cpp
    ```console
    $ cd build
    $ ./surface_normal_visualizer
    ```

## Tips
1. Press `h` key in order to see more controls, such as take screenshot etc.
1. To pick a point in cloud, hold down `SHIFT` key while left-clicking. 
1. In order to change the camera angle, camera pose can also be provided as a part of command line argument as following:
`./advance_visualizer -cam 0.026123,26.123/-0.00599269,0.00252074,1.00064/0.160405,0.457474,-1.54746/0.0138908,-0.984493,-0.17487/0.8575/1863,1176/57,24`

## Issues (or Error Reporting)
Please check [here](https://github.com/ravijo/point-cloud-visualization/issues) and create issues accordingly.
