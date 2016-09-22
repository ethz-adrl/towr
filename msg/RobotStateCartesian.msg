# The state of a robot
# This information must be completely sufficient to reconstruct the whole
# body state q, qd, qdd of the system.

geometry_msgs/Point[4] endeffectors    # the position of each endeffector
geometry_msgs/Pose     base            # The 6D pose of the base