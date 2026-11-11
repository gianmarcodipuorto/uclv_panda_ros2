# Status of the active Panda bridge, published at publish_freq.
string STATE_IDLE="idle"
string STATE_HOLDING="holding"
string STATE_MOVING="moving"
string STATE_STOPPING="stopping"
string STATE_ERROR="error"

std_msgs/Header header
string bridge
string mode
string state
# Internal motion being executed, "stream" while following external commands, empty otherwise
string motion

uint64 cycles
# Control periods longer than 1 ms (packets lost between robot and PC)
uint64 missed_cycles
# Command stream interruptions that triggered a smooth stop
uint64 stalls
# Cycles where the reference could not be commanded as it is (limits, convergence after a deviation)
uint64 limited_cycles
# Cycles of the stream played back by extrapolation, because samples arrived late
uint64 extrapolated_cycles
# Smallest lead of the newest sample over the playback time in the current stream: negative
# values mean samples later than playout_delay_ms
float64 playout_margin_min_ms
uint64 rejected_commands
uint64 dropped_commands

float64 cycle_time_mean_us
float64 cycle_time_max_us
float64 command_rate_hz
float64 command_interval_max_ms
float64 control_command_success_rate
string last_error
