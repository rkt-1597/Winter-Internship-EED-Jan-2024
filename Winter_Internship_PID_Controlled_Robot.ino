#include <NewPing.h>
#include <Servo.h>

#define trigger 2     //Pin of ultrasonic sensor
#define echo 3        //Pin of Ultrasonic Sensor
#define max_dist 200  //Maximum distance limit, in cm

Servo servo;
NewPing sonar(trigger, echo, maximum_distance);

//PID Constant values, obtained experimentally
#define kp 7
#define ki 0.2
#define kd 9

//Variables for Error Calculation
double prevError = 0.0;            //Error of previous step
double integralError = 0.0;        //Error for Integral Term (I Term), of current step

void setup() {
  Serial.begin(9600);
  servo.attach(9);
  servo.write(90);

}
void loop() {
  PID();
  delay(40);
}

int distance() {    //Calculates distance
 unsigned int D = sonar.ping(); 
 int d=D/58;
 
 return d;
}

void PID() {
  int dis = distance ();
  Serial.print(dis);        //For debugging
  Serial.println("cm");

  int setP = 9;             //Setpoint, in cm, from Ultrasonic Sensor
  double error = setP - dis;

  //Calculating PID values
  double Pvalue = error * kp;
  double Ivalue = integralError * ki;
  double Dvalue = (error - prevError) * kd;

  double PIDvalue = Pvalue + Ivalue + Dvalue;
  prevError = error;
  integralError += error;
  Serial.println(PIDvalue);
  int Angle = (int)PIDvalue;

  Angle = map(Angle, -500, 500, 80, 125); //Setting angle of Central Servo Motor

  //Restricting Servo mtor angle within limits, between 80 degrees and 125 degrees, as determined experimentally.
  if (Angle < 80) {
    Angle = 80;
  }
  if (Angle > 125) {
    Angle = 125;
  }

  servo.write(Angle);
  Serial.println(servo.read());
}