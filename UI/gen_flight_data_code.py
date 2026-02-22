# Python script for generating parts of the flight data management logic

flight_packet = """
struct flight_packet_t {
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_yaw;
  float gyro_pitch;
  float gyro_roll;
  float mag_x;
  float mag_y;
  float mag_z;
  float gps_pos_north;
  float gps_pos_west;
  float gps_pos_up;
  float gps_vel_north;
  float gps_vel_west;
  float gps_vel_up;
};
"""

flight_history = ""
flight_update_code = ""

for line in flight_packet.split('\n'):
    if "}" not in line:
        flight_history += line.replace("flight_packet_t", "flight_history_t").replace(";", "[FLIGHT_HISTORY_LENGTH * 2];") + '\n'
    
    if "float" in line:
        param_name = line.strip().split(" ")[1].removesuffix(';')
        flight_update_code += f"FlightHistory.{param_name}[FlightHistory.write_pos] = active_packet.{param_name};\n"
        flight_update_code += f"FlightHistory.{param_name}[FlightHistory.write_pos + FLIGHT_HISTORY_LENGTH] = active_packet.{param_name};\n"

print(flight_history)
print(flight_update_code)