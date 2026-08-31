uint8 PREPARING=0
uint8 RUNNING=1
uint8 FINISHED=2
uint8 PAUSED=3

std_msgs/Header header
uint8 phase
int32 red_score
int32 blue_score
float64 remaining_time
string winner
string reason
