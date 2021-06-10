uint8 motor_mode # 0 for current mode, 1 for velocity mode, 3 for position mode
float64 motor_cmd # current (ma), velocity (rad/sec), or position (rad) cmd
bool[] enable_brake # true to turn on brake, false to turn off
