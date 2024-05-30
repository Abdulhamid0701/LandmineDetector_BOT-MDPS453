float err_theta = 0;
float theta_sp = 0;
float ang_tol = 0.7;
unsigned long lastTime = 0;
const float alpha = 0.98;

float kp_angular = 0.1;
float kd_angular = 0.02;
float ki_angular = 0;
float angleZ = 0;

float kp_vr = 2;
float kp_vl = 2;

float kd_vr = 0.1;
float kd_vl = 0.1;

float ki_vr = 0.1;
float ki_vl = 0.1;

float vright_target;
float vleft_target;

float vright;
float vleft;

float err_vr;
float err_vl;

float prev_err_vr;
float prev_err_vl;
float prev_err_theta = 0;

float tot_err_theta = 0;
float tot_err_vr;
float tot_err_vl;

float target_fwd_speed;

float prev_micris = 0;
float prr = 0;
float prev_micros_choice = 0;

float tNow;
float tPrev = 0;
