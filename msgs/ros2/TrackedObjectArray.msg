# Array of tracked objects with persistent IDs.
# Produced by the detector-agnostic multi_object_tracker node.

std_msgs/Header header
drone_autonomy_msgs/TrackedObject[] objects
string tracker_type       # bytetrack | sort | botsort
string odometry_source    # vslam | mavros | none
