The files in this repository contains all of the arduino and raspberry pi code needed to complete Demo 2 for SEED Lab. 
There are three coding files that represent the different subsystems for this class excpet for localization and simulation and control.
Those files are contained in one arduino file. For computer vision, there is a python file that contains all of the camera initialization and
trajectory functions. The functions in this file are then used in the system integration python file where it takes the camera functions
to get the distance and angle to the aruco marker. These values are then sent to the arduino to be used in the circle and search functions.
