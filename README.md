Dimension-223
=============

**Dimension-223** is a powerful 2d to 3d converter that has auto depth adding capabilities to images. With images provided by user, it automatically generate a depth map for the scene captured in the images and allows the user to view the scene from different perspective and even modify it to some extent.

## Building

You will first need to install the dependencies to build the engine.

### gtkmm

[gtkmm](http://www.gtkmm.org/en/download.shtml) is C++ binding for the GTK+ library, used for building the main user interface of Dimension-223. Note that since the application uses OpenCV which uses Gtk+ version 2.0, same version of gtkmm should be used.

    # For Debian systems, you can install it with apt-get
    sudo apt-get install libgtkmm-2.4-dev

### SFML

[SFML](http://www.sfml-dev.org/download.php) provides simple cross platform interface for rendering with OpenGL. A custom SFML widget is used to integrate SFML with gtkmm window to render OpenGL in our user interface.

    # For Debian systems, you may install it with apt-get
    sudo apt-get install libsfml-dev

### GLM

[GLM](http://glm.g-truc.net/0.9.7/index.html) is OpenGL Mathematics library containing header only implementation based on GLSL specifications.

    # For Debian systems, you may install it with apt-get
    sudo apt-get install libglm-dev


### GLEW

[GLEW](http://glew.sourceforge.net/index.html) provides efficient extension loading library for OpenGL applications.

    # For Debian systems, you may install it with apt-get
    sudo apt-get install libglew-dev


### OpenCV
[OpenCV](http://opencv.org/downloads.html) is a powerful image processing library.

    # For Debian systems, you may install it with apt-get
    sudo apt-get install libopencv-dev

### Building Dimension-223

Simply execute `make` to build the software, which is then built into the *bin* directory. If error is encountered, try to clean up the previous build outputs using `make clean`.

    make clean
    make

You can then simply run the output software from the bin directory.

    bin/d223


### Building Docs

You will need `doxygen` to generate the documentation files. Simply run it with the *Doxyfile* present in the project directory. The generated documentation will be placed inside the *docs* directory.

    doxygen Doxyfile
