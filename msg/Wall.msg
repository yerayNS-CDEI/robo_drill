# A single detected planar wall segment, expressed in the parent WallArray
# header frame (a fixed, gravity-aligned frame, e.g. odom).

geometry_msgs/Point start      # wall base endpoint A (at z_min)
geometry_msgs/Point end        # wall base endpoint B (at z_min)
geometry_msgs/Vector3 normal   # unit outward normal (horizontal); standoff direction

# Plane equation: normal.x*x + normal.y*y + normal.z*z + d = 0
float32 d

float32 length                 # along-wall horizontal extent (m)
float32 height                 # z_max - z_min (m)
float32 z_min
float32 z_max

float32 confidence             # 0..1 (support density / persistence)
int32 inliers                  # supporting points
bool refined                   # true if the constrained 3D plane fit succeeded

# Tier-C only: centers of door/window openings lying on this wall (same frame).
# Empty from the detector; filled by wall_mapping_node when opening fusion is
# enabled (requires the YOLO hand-eye calibration to be valid in the map frame).
geometry_msgs/Point[] openings