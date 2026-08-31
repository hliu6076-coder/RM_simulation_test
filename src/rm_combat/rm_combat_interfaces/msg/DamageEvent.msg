uint8 PROJECTILE=0
uint8 OBSTACLE_COLLISION=1
uint8 ROBOT_COLLISION=2

std_msgs/Header header
uint64 event_id
uint8 source
string attacker_id
string target_id
int32 damage
int32 remaining_hp
string collision_name
float64 relative_speed
