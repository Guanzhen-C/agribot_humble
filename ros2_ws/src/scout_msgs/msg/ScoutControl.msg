builtin_interfaces/Time stamp

# Range 0 to 1, 1 is max throttle
float64 throttle
# Range 0 to 1, 1 is max brake
float64 brake
# Range -1 to +1, +1 is maximum left turn
float64 steering

uint8 NO_COMMAND=0
uint8 NEUTRAL=1
uint8 FORWARD=2
uint8 REVERSE=3

uint8 gearshift

# True when the ENABLE_TURBO button is pressed
bool enable_turbo
