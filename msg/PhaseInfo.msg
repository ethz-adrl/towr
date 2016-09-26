# Contains the contact configuration of one specific phase

int32   n_completed_steps   # how many steps have been taken prior to this phase
Contact[]   free_contacts   # these are not fixed in advanced, but optimized over
xpp_msgs/Foothold[] fixed_contacts   # these are determined by the start stance
int32                  id   # A unique identifier for each phase
float64          duration   # How long this phase lasts