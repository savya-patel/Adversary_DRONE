Author: Ashley Ortega

YOLOv4 Tiny project followed this YouTube video: https://youtu.be/jYtc5XTYLP0?si=9rpVTewHRZ-pmyl8
Was parallel workstream to YOLOv5 not function on Jetson yet made to replace YoloV5 with less computaionally hungry model.

NON FUNCTIONAL ON JETSON WILL WORK ON RPI4

In the yolov4tinyrpi folder, the test.py is edited to match the name of the weights that are received when customly trained. In this folder, yolov4-tiny-customer_4000.weights was created after training Drone images following the youtube video repository.

To run the test.py file with the pretrained weights:

SSH into RasperryPi:
1. On VSCode, connect to the Raspberry PI via
ssh eagle@10.251.75.235
2. Enter password.

To activate the virtual environment:
1. Enter in VSCode terminal
Bash:
$ eagle@eagle: cd Adversary_DRONE
$ eagle@eagle:~/Adversary_DRONE source venv/bin/activate
(venv) eagle@eagle:~/Adversary_DRONE

To run the test.py file:
1. On VSCode
Bash:
(venv) eagle@eagle:~/Adversary_DRONE/
(venv) eagle@eagle:~/Adversary_DRONE/ $cd yolov4
(venv) eagle@eagle:~/Adversary_DRONE/yolov4/ $ cd yolov4tinypi4
(venv) eagle@eagle:~/Adversary_DRONE/yolov4/yolov4tinypi4 $python3 test.py
