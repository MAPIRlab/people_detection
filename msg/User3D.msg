# Name of the user (if recognized)
string name

# Position+Orientation 3D coordinates of the face (if known)
geometry_msgs/Pose pose_3D

# Reliabity of the user search (related to the consolidation scheme)
float32 certainty

# Body parts in 3D
BodyPart3D[] body_part_3d

## height
uint8 human_height

## valid orientation
uint8 valid_orientation
