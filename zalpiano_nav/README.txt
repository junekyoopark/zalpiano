Nav2 requires the following transformations to be published in ROS:

    map => odom

    odom => base_link

    base_link => base_laser (sensor base frames) (so in our case, wheel encoder? odometry?)
