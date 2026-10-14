# A velocity belongs to one execution; consumers must reject old/invalid values.
builtin_interfaces/Time stamp
string frame_id
uint64 producer_session
uint64 generation
uint64 update_sequence
bool valid
geometry_msgs/Twist velocity

# True only after execution path and controller histories have been cleared.
bool control_history_cleared
