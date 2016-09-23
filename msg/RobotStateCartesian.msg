# The state of a robot
# This information must be completely sufficient to reconstruct the whole
# body state q, qd, qdd of the system.

StateLin3d[4]   endeffectors    # the position of each endeffector in World Frame
bool[4]        ee_in_contact    # Defines for each ee if it is touching the environement
BaseState               base    # The 6D pose of the base