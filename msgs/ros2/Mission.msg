# Mission Message
std_msgs/Header header
string mission_id
string mission_type
geometry_msgs/Pose[] waypoints

# Survey mission fields (mission_type == "survey"), CAP-001 / DES-003
geometry_msgs/Polygon survey_polygon   # vertices in local ENU frame, z ignored
float32 survey_altitude_m              # AGL altitude for the survey
float32 forward_overlap                # 0.0-1.0, default 0.75
float32 side_overlap                   # 0.0-1.0, default 0.60
float32 survey_speed_ms                # planned ground speed, default 5.0
float32 capture_rate_hz                # recorder capture rate; 0 = recorder default (2.0)
