#define FL_MOTOR_SPEED_PIN 3
#define FL_ENCODER_PHASE_A 2
#define FL_ENCODER_PHASE_B 4

#define FR_MOTOR_SPEED_PIN 5
#define FR_ENCODER_PHASE_A 7
#define FR_ENCODER_PHASE_B 8

#define RL_MOTOR_SPEED_PIN 6
#define RL_ENCODER_PHASE_A 10
#define RL_ENCODER_PHASE_B 11

#define RR_MOTOR_SPEED_PIN 9
#define RR_ENCODER_PHASE_A 12
#define RR_ENCODER_PHASE_B 13

#define LEFT_SIDE_MOTORS_DIR_PIN 0
#define RIGHT_SIDE_MOTORS_DIR_PIN 1


unsigned int fl_encoder_counter = 0;
unsigned int fr_encoder_counter = 0;
unsigned int rl_encoder_counter = 0;
unsigned int rr_encoder_counter = 0;

String fl_encoder_sign = "p";
String fr_encoder_sign = "p";
String rl_encoder_sign = "p";
String rr_encoder_sign = "p";

double fl_wheel_measured_vel = 0.0;
double fr_wheel_measured_vel = 0.0;
double rl_wheel_measured_vel = 0.0;
double rr_wheel_measured_vel = 0.0;

bool is_fl_wheel_cmd = false;
bool is_fr_wheel_cmd = false;
bool is_rl_wheel_cmd = false;
bool is_rr_wheel_cmd = false;

void setup() {

  pinMode(FL_MOTOR_SPEED_PIN,OUTPUT);
  pinMode(FR_MOTOR_SPEED_PIN,OUTPUT);
  pinMode(RL_MOTOR_SPEED_PIN,OUTPUT);
  pinMode(RR_MOTOR_SPEED_PIN,OUTPUT);

  pinMode(LEFT_SIDE_MOTORS_DIR_PIN,OUTPUT);
  pinMode(RIGHT_SIDE_MOTORS_DIR_PIN,OUTPUT);

  pinMode(FL_ENCODER_PHASE_B,INPUT);
  pinMode(FR_ENCODER_PHASE_B,INPUT);
  pinMode(RL_ENCODER_PHASE_B,INPUT);
  pinMode(RR_ENCODER_PHASE_B,INPUT);

  attachInterrupt(digitalPinToInterrupt(FL_ENCODER_PHASE_A),FLEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_ENCODER_PHASE_A),FREncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(RL_ENCODER_PHASE_A),RLEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(RR_ENCODER_PHASE_A),RREncoderCallback, RISING);

  digitalWrite(LEFT_SIDE_MOTORS_DIR_PIN,LOW);
  analogWrite(FL_MOTOR_SPEED_PIN, LOW);
  analogWrite(FR_MOTOR_SPEED_PIN, LOW);

  digitalWrite(RIGHT_SIDE_MOTORS_DIR_PIN,LOW);
  analogWrite(RL_MOTOR_SPEED_PIN, LOW);
  analogWrite(RR_MOTOR_SPEED_PIN, LOW);

  Serial.begin(115200);
}

void loop() {

  if(Serial.available()){
    char chr = Serial.read();
    if(chr == '')
  }

  wheel_measured_vel = 10 * encoder_counter * ((3.0 * 60.0)/(7.0 * 200.0)) * 0.10472;
  String encoder_read = encoder_sign + String(wheel_measured_vel);
  Serial.println(encoder_read);
  analogWrite(MOTOR_SPEED_PIN,100);
  encoder_counter = 0;



  delay(100);
}

void FLEncoderCallback()
{
  fl_encoder_counter++;
  if(digitalRead(FL_ENCODER_PHASE_B) == HIGH){
    // clockwise
    fl_encoder_sign = "p";
  }
  else{
    // counter-clockwise
    fl_encoder_sign = "n";
  }
}

void FREncoderCallback()
{
  fr_encoder_counter++;
  if(digitalRead(FR_ENCODER_PHASE_B) == HIGH){
    // clockwise
    fr_encoder_sign = "p";
  }
  else{
    // counter-clockwise
    fr_encoder_sign = "n";
  }
}

void RLEncoderCallback()
{
  rl_encoder_counter++;
  if(digitalRead(RL_ENCODER_PHASE_B) == HIGH){
    // clockwise
    rl_encoder_sign = "p";
  }
  else{
    // counter-clockwise
    rl_encoder_sign = "n";
  }
}

void RREncoderCallback()
{
  rr_encoder_counter++;
  if(digitalRead(RR_ENCODER_PHASE_B) == HIGH){
    // clockwise
    rr_encoder_sign = "p";
  }
  else{
    // counter-clockwise
    rr_encoder_sign = "n";
  }
}
