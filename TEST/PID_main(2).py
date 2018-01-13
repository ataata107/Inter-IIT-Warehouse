#PID
pid_T = PIDController(proportional = 0.15, derivative_time = 10, integral_time=0)
pid_R = PIDController(proportional = 0.15, derivative_time = 10, integral_time=0)
pid_P = PIDController(proportional = 0.15, derivative_time = 10, integral_time=0)
pid_Y = PIDController(proportional = 0.15, derivative_time = 10, integral_time=0)
pid_T.vmin, pid_T.vmax = -0.05, 0.05
pid_R.vmin, pid_R.vmax = -1, 1
pid_P.vmin, pid_P.vmax = -1, 1
pid_Y.vmin, pid_Y.vmax = -1, 1
pid.setpoint_T = 1.0   #aTargetAltitude(m)
TAltitude = pid.setpoint_T
pid.setpoint_R=0 #offset
TRoll_Setpoint=pid.setpoint_R
pid.setpoint_P=0 #
TPitch_Setpoint=pid.setpoint_P
pid.setpoint_Y=0 #
TYaw_Setpoint=pid.setpoint_Y
baseThrust = 0.55
baseRoll = 0
basePitch=0
baseYaw=0

