# AI-project

Instruction:
1. Follow the instruction in the sb_web_app folder to start a HTTP and ROS server within your ROS setup for survivor buddy. Turn on the Camera.
2. Run 'python3 sb_project.py'.

Description:
The file sb_project.py has the implementation of 4 Behaviors - Attention, Mirroring Face, Youtube Interaction and Dance.

Observer pattern has been used to implement this. Its brief working is explained below: 
1. In this a subject like Camera Perception will notify its observers when it gets a new image. 
2. The notification from the subject updates the values at the observers like Face Cartographer to process the image for face recognition.
