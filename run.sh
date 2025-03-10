sudo docker build . --tag ros_image
sudo docker run -it --rm -v ~/ros-test/:/ros-test/:rw ros_image

