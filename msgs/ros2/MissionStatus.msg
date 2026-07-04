# Mission Status Message — DES-003 (CAP-001 WP-1)
std_msgs/Header header
string mission_id
string state        # "active" | "complete" | "aborted" | "rejected"
string detail       # human-readable reason (rejection cause, abort source)
