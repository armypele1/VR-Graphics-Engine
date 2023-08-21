# VR Graphics Engine in Python
I extended a rudimentary graphics engine (https://github.com/ecann/RenderPy) to support real-time rendering, tracking, physics and distortion pre-correction for Virtual Reality. The focus of this project was to better understand how virtual reality engines work, and so little effort went towards improving performance - all calculations are performed on the CPU.

![cow](https://raw.githubusercontent.com/armypele1/VR-Graphics-Engine/main/image1.png)

## To Run
```python 
python3 render.py
```
## My Additions to the Engine

I have tried to make my code easily readable without documentation so keep my explanations quite brief here.

### Vector.py
A pretty standard vector class that is able to handle homogeneous coordinates.

### Matrix.py
In order to make transformations more straightforward, I implemented my own Matrix class which is able to work in tandem with my vector class. Contains useful matrix operations such as multiplication, getting transpose, inverse and more.

### Quaternion.py
My quaternion class is used to avoid a Gimbal Lock scenario. Can be used to convert to and from: Euler angles, axis angle representation or rotation matrices.

### Transformations.py
All transformation matrices such as rotation, scaling, translation, perspective projection, etc can be found in transformations.py.

### Camera.py
This essentially just stores information about the camera in a more intuitive format, allowing for improved code readability.

### DataManager.py
This contains all code responsible for reading and handling IMU data. This includes functions that perform gyro integration and tilt correction. I found an alpha value of 0.01 to be sufficient for correcting drift. As I increased the alpha value toward 0.1, I found that it severely limited the rotation of the headset instead of just correcting drift. To improve tilt correction, I averaged acceleration values over the surrounding 100 data points in order to produce a better estimate for the gravity vector. I also removed data points that had a significant difference from the last and previous data points, as these points were likely heavily influenced by player movement.

### PhysicsEngine.py
I implemented a simple physics engine by defining a new class PhysicsObject in physicsEngine.py. Instances of this class essentially act as the physics-based counterpart to all models in the scene, allowing for gravity-based movement and simple collision detection. Collisions are handled by calculating the total collision velocity and then defining two impulse forces that consider the colliding objects' relative masses. Then, each impulse force is applied to each object in the direction of the collision normal.

### Render.py
I completely overhauled render.py to integrate all of the additions described above. I opted to use OpenCV to enable real-time output of the frame buffer to the screen. You can see the complete render pipeline for perspective projection laid out in this file, which I find to be very useful for understanding how it works. Note that I also implemented distortion pre-correction inside render.py. I set c1=0.2, and c2=0.2, as
I found this displayed a suitable amount of the effect without being overwhelming.


## Modules From RenderPy

### Image.py
Contains an image class capable of generating an image and exporting it to a PNG. Images are implemented as a buffer of 32-bit RGBA pixel color data stored in a byte array. This modules uses `zlib` and `struct` for compressing and packing PNG data. 

### Shape.py
Classes representing points, lines and triangles. Each has a `draw()` method for drawing that shape in an Image. Anti-aliased lines are drawn using [Wu's Line Drawing Algorithm](https://en.wikipedia.org/wiki/Xiaolin_Wu's_line_algorithm). Triangles are drawn by iterating over a bounding box and calculating barycentric coordinates to smoothly interpolate color over the shape.

### Model.py
Class with functions for reading a `.obj` file into a stored model, retrieving vertices, faces, properties of the model.
