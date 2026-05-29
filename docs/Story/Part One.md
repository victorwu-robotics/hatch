## Part One: Seeing the Robot

I cannot debug what I cannot see. A robot is a physical thing — it has shape, it occupies space, it moves. If I send a command and the robot moves wrong, I need to see what happened. Not read log messages. Not inspect joint angles in a terminal. I need to see the arm, the torch, the camera, the scanner — all of them moving together in space, exactly as they would on the factory floor.

This is not a luxury. It is a prerequisite for understanding. Without vision, I am programming blind. With vision, the robot's behavior becomes obvious. The torch dipped because the wrist rotated unexpectedly. The scanner missed the groove because the arm approached from the wrong angle. The point cloud is sparse because the camera is too far from the surface. These are not things I deduce from numbers. They are things I see.

So the first component Hatch needed was a **visualizer**. A 3D view that shows every link of the robot, every sensor mounted to its wrist, every tool it holds. Not a separate simulation — a window into the robot's internal state. The visualizer does not control anything. It observes. It reflects whatever the kinematic model tells it is true. If the model says the elbow is at 45 degrees, the visualizer shows the elbow at 45 degrees. If the model is wrong, the visualizer shows the wrong thing — and I know the model needs fixing.

This is Principle #3: **Visualizer as Mind-Prying Tool.** The 3D view is not the robot. It is what the robot thinks it is. The gap between them is where debugging happens.

But to show the robot, the visualizer needs something to show. It needs the robot's shape — the meshes, the geometry, the physical form of every link. And it needs to know where each piece goes. That requires a description of the robot. That requires a URDF.

