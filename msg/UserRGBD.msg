# Name of the user (if recognized)
string name

# [u,v,w,h] X and Y image coordinates of the center of the face (u,v) + width and height of the face (w,h)
float32 u
float32 v
float32 w
float32 h

# Position+Orientation 3D coordinates of the face (if known)
geometry_msgs/Pose pose_3D

# Reliabity of the user search (related to the consolidation scheme)
float32 certainty

# Body parts in 3D
BodyPart3D[] body_part_3d
