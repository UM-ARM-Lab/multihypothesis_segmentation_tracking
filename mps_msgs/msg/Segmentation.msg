# Convenience class for passing image segmentation results
sensor_msgs/CameraInfo camera_info
sensor_msgs/Image rgb
sensor_msgs/Image depth
sensor_msgs/Image segments # Image containing per-pixel segment IDs
sensor_msgs/Image ucm      # Image containing the ultrametric contour map of the segmentation