#######################
Building your robot
#######################

Now you know how to make your own node and setup ROS, lets use this knowledge to build a basic robot

.. note::
    Work in progress!

    If you are reading this and **June of 2025** has passed, then I probably am not going to finish this section ever — and now, this is your responsibility to complete. 😅
    If it is not then **find me** and motivate me to complete this section!

    Here is an outline of what to do:
        - Learn about MicroROS and how to run it on ESP32, `link to resources <https://github.com/micro-ROS/micro_ros_espidf_component>`_
        - Once that is done make the microcontroller subscribe to ``cmd_vel`` topic and using the values recieved from their actuate the motors.
        - Make the microcontroller publish the odometry of the robot to a topic (odometry: speed and distance of the robot), you will need a motor with encoder to achieve this.
        - Do this and the basic robot is done! now it's time for some documentation, for that go and `read this section <https://github.com/Robotics-PEC/Getting-Started-with-ROS/tree/main/docs#readme>`_
        - Done all this? Hooray! 🙌 now you have a very fancy RC car you can play with.