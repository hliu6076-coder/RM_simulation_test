uint8 MISS=0
uint8 HIT=1
uint8 BLOCKED=2

std_msgs/Header header
uint64 shot_id
string shooter_id
uint8 outcome
string target_id
string armor_id
string collision_name
geometry_msgs/Point origin
geometry_msgs/Point endpoint
float64 distance
