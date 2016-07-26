Dimension-223
=============

**Dimension-223** is a powerful 2d to 3d converter that has auto depth adding capabilities to images. With images provided by user, it automatically generate a depth map for the scene captured in the images and allows the user to view the scene from different perspective and even modify it to some extent.

## Building

You will first need to install the dependencies to build the engine.

### SFML

[SFML](http://www.sfml-dev.org/download.php) provides simple cross platform interface for rendering with OpenGL. A custom SFML widget is used to integrate SFML with gtkmm window to render OpenGL in our user interface.

```bash
$ sudo apt-get install libsfml-dev
```

### GLM

[GLM](http://glm.g-truc.net/0.9.7/index.html) is OpenGL Mathematics library containing header only implementation based on GLSL specifications.

```bash
$ sudo apt-get install libglm-dev
```


### GLEW

[GLEW](http://glew.sourceforge.net/index.html) provides efficient extension loading library for OpenGL applications.

```
$ sudo apt-get install libglew-dev
```


### Gtkmm

TODO


### OpenCV

TODO

### Point Cloud Library

TODO


### Bullet

TODO


### Building Dimension-223

You need `cmake` to build the application. From inside the project directory, run:

```bash
$ mkdir build
$ cd build
$ cmake ..
$ make
```

The application `d223` is then built in the *build* directory.


### Building Docs

You will need `doxygen` to generate the documentation files. Simply run it with the *Doxyfile* present in the project directory. The generated documentation will be placed inside the *docs* directory.

```bash
$ doxygen Doxyfile
```
