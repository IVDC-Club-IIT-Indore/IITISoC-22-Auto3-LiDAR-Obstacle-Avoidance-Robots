# The name of the waypoint or path
string name
# If string not given, can playback by index instead
int16 index

# Playback modes
int16 GO_TO_WAYPOINT = 0 # Go to a named or numbered waypoint
int16 GO_TO_PATH_START = 1 # Go to the start point of the path
int16 PLAY_PATH = 2 # Play path at offset & reset offset
int16 NUM_MODES = 3 # Total number of available modes; this is an invalid mode selection!
# Select playback mode
int16 mode
