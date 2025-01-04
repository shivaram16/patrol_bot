#define MOTOR_SPEED_PIN 9
#define MOTOR_DIR_PIN 8
#define ENCODER_PHASE_A 3
#define  ENCODER_PHASE_B 4

unsigned int encoder_counter = 0;
String encoder_sign = "p";
double wheel_measured_vel = 0.0;

void setup() {
  pinMode(MOTOR_SPEED_PIN,OUTPUT);
  pinMode(MOTOR_DIR_PIN,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PHASE_A),EncoderCallback, RISING);

  digitalWrite(MOTOR_DIR_PIN,LOW);
  analogWrite(MOTOR_SPEED_PIN, LOW);
  Serial.begin(115200);
}

void loop() {

  wheel_measured_vel = 10 * encoder_counter * ((3.0 * 60.0)/(7.0 * 200.0)) * 0.10472;
  String encoder_read = encoder_sign + String(wheel_measured_vel);
  Serial.println(encoder_read);
  analogWrite(MOTOR_SPEED_PIN,100);

  encoder_counter = 0;
  delay(100);
}

void EncoderCallback()
{
  encoder_counter++;
  if(digitalRead(ENCODER_PHASE_B) == HIGH){
    // clockwise
    encoder_sign = "p";
  }
  else{
    // counter-clockwise
    encoder_sign = "n";
  }
}
