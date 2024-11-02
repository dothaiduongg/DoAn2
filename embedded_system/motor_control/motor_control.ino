#include<PID_v1.h>
#include <Arduino_FreeRTOS.h>
#define encoderA1 2
#define encoderB1 8
#define encoderA2 3
#define encoderB2 7
#define pwm1 10
#define pwm2 9
#define pwm3 6
#define pwm4 5
#define LOOPTIME 10
#define ROBOT_MOTOR_PPR 100
#define ROBOT_WHEEL_RADIUS 0.725f
#define ROBOT_WHEEL_SEPARATION 0.26f
#define sample_time_ms 10
#define pid_rate float(sample_time_ms) / 1000.0f
#define ROBOT_MAX_LINEAR_M_S 0.2
#define ROBOT_MIN_LINEAR_M_S -0.2
#define ROBOT_MAX_ANGULAR_R_S 2.0
#define ROBOT_MIN_ANGULAR_R_S -2.0
double kp1=12.2, ki1=100, kd1=0.01;
double kp2=11, ki2=100, kd2=0.01;
double input1 = 0, output1 = 0, setpoint1 = 0;
double input2 = 0, output2 = 0, setpoint2 = 0;
unsigned long lastTime, now;
unsigned long lastTime2, now2;
volatile long encoderPos1 = 0, last_pos1 = 0, lastpos1 = 0;
volatile long encoderPos2 = 0, last_pos2 = 0, lastpos2 = 0;
double l_pos, r_pos;
unsigned long l_ticks_prev, r_ticks_prev;
double x_pos, y_pos, theta, v, w; 
double l_speed, r_speed;
double linear, angular;
int32_t dl_ticks, dr_ticks;
String inByte; 
PID myPID(&input1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);
PID myPID2(&input2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT);

void update_pid(int32_t l_encoder_ticks, int32_t r_encoder_ticks ) {
    int32_t l_ticks = l_encoder_ticks;
    int32_t r_ticks = r_encoder_ticks;
    l_pos = (2*PI)*l_ticks/ROBOT_MOTOR_PPR;
    r_pos = (2*PI)*r_ticks/ROBOT_MOTOR_PPR;
    dl_ticks = l_ticks - l_ticks_prev;
    dr_ticks = r_ticks - r_ticks_prev;
    updateOdometry(dl_ticks, dr_ticks);
    l_speed = (2.0 * PI) * dl_ticks / (ROBOT_MOTOR_PPR * pid_rate);
    r_speed = (2.0 * PI) * dr_ticks / (ROBOT_MOTOR_PPR * pid_rate);
    v = (ROBOT_WHEEL_RADIUS / 2.0f) * (l_speed + r_speed);
    w = (ROBOT_WHEEL_RADIUS / ROBOT_WHEEL_SEPARATION) * (r_speed - l_speed);
    input1 = l_speed;
    input2 = r_speed;
    myPID.Compute();
    myPID2.Compute();
    pwmOut1(output1);
    pwmOut2(output2);
    l_ticks_prev = l_ticks;
    r_ticks_prev = r_ticks;
}
void updateOdometry(int32_t dl_ticks, int32_t dr_ticks) {
    double delta_l = (2 * PI * ROBOT_WHEEL_RADIUS * dl_ticks) / ROBOT_MOTOR_PPR;
    double delta_r = (2 * PI * ROBOT_WHEEL_RADIUS * dr_ticks) / ROBOT_MOTOR_PPR;
    double delta_center = (delta_l + delta_r) / 2;
    x_pos += delta_center * cos(theta);
    y_pos += delta_center * sin(theta);
    theta += (delta_r - delta_r)/ROBOT_WHEEL_SEPARATION;
}
void setUnicycle(double v, double w) {
    //limit value
    if(v > ROBOT_MAX_LINEAR_M_S) v = ROBOT_MAX_LINEAR_M_S;
    if(v < ROBOT_MIN_LINEAR_M_S) v = ROBOT_MIN_LINEAR_M_S;
    if(w > ROBOT_MAX_ANGULAR_R_S) w = ROBOT_MAX_ANGULAR_R_S;
    if(w < ROBOT_MIN_ANGULAR_R_S) w = ROBOT_MIN_ANGULAR_R_S;
    double v_l = (2 * v - w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS);
    double v_r = (2 * v + w * ROBOT_WHEEL_SEPARATION) / (2 * ROBOT_WHEEL_RADIUS);
    linear = v;
    angular = w;
    setWheel(v_l, v_r);
}
void setWheel(double left_speed, double right_speed) {
    setpoint1 = left_speed;
    setpoint2 = right_speed;
}
void publishData() {
    Serial.print(x_pos); Serial.print("/");
    Serial.print(y_pos); Serial.print("/");
    Serial.print(theta); Serial.print("/");
    Serial.print(l_speed); Serial.print("/");
    Serial.print(r_speed); Serial.print("/");
    Serial.print(output1); Serial.print("/");
    Serial.print(output2); Serial.print("/"); 
}
void getData() {
    int leng, j;
    String data, data2;
    while(Serial.available()) {
        inByte = Serial.readString();leng = inByte.length();
        for(int i=0; i<leng; i++) {
            if(inByte[i]!='/') {
                data+=inByte[i];
            }
            else {
                data+= ' ';
                j = i+1; 
            }
        }
        for(int i=j; i<leng; i++) 
        {
            data2 += data[i];
            data[i] = ' ';
        }
        linear = data.toDouble();
        angular = data2.toDouble();
        publishData();
    }
}
void setup() {
// put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(encoderA1, INPUT_PULLUP);
    pinMode(encoderB1, INPUT_PULLUP);
    pinMode(encoderA2, INPUT_PULLUP);
    pinMode(encoderB2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderA1), ISR_encoderA1, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoderA2), ISR_encoderA2, FALLING);
    pinMode(pwm1, OUTPUT);
    pinMode(pwm2, OUTPUT);
    pinMode(pwm3, OUTPUT);
    pinMode(pwm4, OUTPUT);
    TCCR1B = TCCR1B & 0b11111000 | 1; // set 31KHz PWM to 
    prevent motor noise
    TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); 
    TCCR0B = _BV(CS00); 
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(1);
    myPID.SetOutputLimits(-255, 255);
    myPID2.SetMode(AUTOMATIC);
    myPID2.SetSampleTime(1);
    myPID2.SetOutputLimits(-255, 255);
}
void loop() {
    now = millis();
    if(now - lastTime > LOOPTIME)
    {
        lastTime = now;
        getData();
        setUnicycle(linear, angular);
        update_pid(encoderPos1, encoderPos2);
        updateOdometry(dl_ticks, dr_ticks);
    }
}
void pwmOut1(int out) {
    if(out > 0) {
        analogWrite(pwm1, 0);
        analogWrite(pwm2, abs(out));
    }
    if(out < 0) {
        analogWrite(pwm1, abs(out));
        analogWrite(pwm2, 0);
    }
}
void pwmOut2(int out) {
    if(out > 0) {
        analogWrite(pwm3, 0);
        analogWrite(pwm4, abs(out));
    }
    if(out < 0) {
        analogWrite(pwm3, abs(out));
        analogWrite(pwm4, 0);
    }
}
void ISR_encoderA1() {
    bool PinB = digitalRead(encoderB1);
    bool PinA = digitalRead(encoderA1);
    if (PinB == LOW) {
        if (PinA == HIGH) {
            encoderPos1++;
        }
        else {

            encoderPos1--;
        }
    }
    else {
        if (PinA == HIGH) {
            encoderPos1--;
        }
        else {
            encoderPos1++;
        }
    }
}
void ISR_encoderA2() {
    bool PinB = digitalRead(encoderB2);
    bool PinA = digitalRead(encoderA2);
    if (PinB == LOW) {
        if (PinA == HIGH) {
            encoderPos2--;
        }
        else {
            encoderPos2++;
        }
    }
    else {
        if (PinA == HIGH) {
            encoderPos2++;
        }
        else {
            encoderPos2--;
        }
    }
}