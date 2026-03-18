# A single tracked object with a persistent ID across frames.
# Produced by the detector-agnostic multi_object_tracker node.

int32 track_id               # Persistent track ID assigned by the tracker
string class_id              # Class label (string, detector-agnostic)
float32 score                # Detection confidence at last association
vision_msgs/BoundingBox2D bbox  # Bounding box in image pixel coordinates
int32 age                    # Number of frames this track has been alive
geometry_msgs/Vector3 velocity  # Estimated velocity (pixels/s, z unused)
