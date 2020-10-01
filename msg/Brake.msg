float32 setpoint
float32 actual
float32 delta
float32 linear_encoder # The output of the linear encoder
float32 R_IS
float32 L_IS
float32 R_current # (A)
float32 L_current # (A)
int32 pwm # The PWM command
int32 state # 0: IDLE || 1: PULL || -1: RELEASE
