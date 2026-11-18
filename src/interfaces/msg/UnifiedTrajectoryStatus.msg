# CTRL-R4d: trajectory lifecycle status for a specific request_id.
uint8 IDLE=0
uint8 ACCEPTED=1
uint8 RUNNING=2
uint8 SETTLED=3
uint8 REJECTED=4
uint8 CANCELLED=5
uint8 FAILED=6

uint32 request_id
uint8 status
float64 ex
float64 ey
float64 ez
float64 vh
float64 vz
string message
