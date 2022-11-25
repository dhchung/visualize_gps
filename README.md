I used https://www.geoplaner.com/ to get the longitude and latitude from the map

Choose four points (A, B, C, D)

Enter each latitude and longitude in /satellite_img/sat_coord.txt with tab delimited manner

Capture the image at geoplaner site and save it in /satellite_img/img.png

1. Estimate the homography between UTM coordinates and image coordinates
    - run visualize_gps_estimate_homography: homography.cpp
    - this will save the homography matrix which transforms the utm coordinates to the image coordinate

2. Run ros node
    - change the topic name at visualize_gps_node.cpp line 52 to whatever topic you have
    - Topic type is sensor_msgs::NavSatFix

<p align='center'>
    <img src="./doc/demo.gif" alt="drawing" width="400"/>
</p>
