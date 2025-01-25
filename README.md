<img src=data/doom_ros.png width="320px"/>

# doom_ros

Doom can run anywhere. Now it runs on ROS too.

<p float="left">
<img src=data/doom_lvl1.gif width="320px"/>
<img src=data/doom_rqt_image_view.png width="320px"/>
</p>

# Running it

`ros2 launch doom_ros doom_ros.launch.py`

# Notes

- The package has been tested with the following controllers:
  - 8bitDo SN30+ Pro
  - Logitech F710
- In case you use a controller that has not been tested and the inputs are off, consider making a PR :)
- Extra files on `data` folder:
  - Repository resources, videos and images
  - `DOOM1.WAD` file
  - A rosbag (`doom_rosbag.tar.gz`) with the first level's gameplay, including image and joy inputs. (same as the video provided)

# Disclaimer

This repository uses the **shareware** version of `doom.wad`. As far as I can tell, this version can be freely re-distributed. If something changes in the future or I have misunderstood how this file can be used, just request a removal!

<sub>Don't sue me please! :)</sub>
