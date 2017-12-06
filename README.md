This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car using ROS. More information can be found from project introduction page [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Following are the team members of Zoom Ahead:

1) Girish Pai - team lead (girish.salute@gmail.com)
2) JC Li - jincheng.li@gmail.com
3) Chun Pook - pookc@hotmail.com
4) Munir Jojo - munirjojoverge@yahoo.es

NOTE to reviewer:

As the Neural Network trained frozen model .pb file is ~200MB, too large to be uploaded to github.com. The file can be downloaded [here](https://drive.google.com/file/d/17mtiBbiMi5iVYtQQwY0GPGEieXRnFiUi/view?usp=sharing), and please put the file at <tot>/ros/src/tl_detector/frozen_inference_graph.pb so that it can be loaded correctly.


A recorded rosbag playback with this NN Traffic Light classifier working can be seen here:
[![NN TL classifier with ROS bagfile](https://img.youtube.com/vi/E0k9DKLc9aU/0.jpg)](https://youtu.be/E0k9DKLc9aU "! Click to watch on YouTube.")


## System introduction

### Diagram

The final project diagram is illustrated below. It consists of four major components: perception, planning, control/execution and environment(Udacity Car Simulator). The whole system operates on ROS and there are several ROS nodes in each of the first three components. 

![System diagram](./imgs/final-project-ros-graph-v2.png)

Our team implemented two versions of traffic light classifiers - one for simulation and the other one for real scanario.

- Simulation - Traditional computer vision based classifier.

- Neural Network based classifier - which is far more robust and would work well in real scenario.

More information about the NN based classifier and the system in general can be found in one of our teammates JC Chen's [github repo](https://github.com/jinchenglee/CarND-Capstone) :





