# The PoseStamped is required if executetype=EXECUTETYPE_REPLACE , it represents the position in the header frame_id and is the path startpose
uint8 EXECUTETYPE_REPLACE = 0
uint8 EXECUTETYPE_APPEND = 1
uint8 EXECUTETYPE_LOCAL = 2
uint8 executetype
GeographicPosition georeference
geometry_msgs/Pose pathstartpose
PathSection[] sections
