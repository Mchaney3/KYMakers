// Pull code from the RobotCar example for ESP32 under iBusBM Examples

/*
 * L298N Motor 1 Pin IN1 = Mega Digital Pin 2
 * L298N Motor 1 Pin IN2 = Mega Digital Pin 3
 * L298N Motor 1 Pin PWM (Speed Control) = Mega Digital Pin 6
 * 
 * L298N Motor 2 Pin IN3 = Mega Digital Pin 4
 * L298N Motor 2 Pin IN4 = Mega Digital Pin 5
 * L298N Motor 2 Pin PWM (Speed Control) = Mega Digital Pin 7
 */

#define Pin_D1_R  2
#define Pin_D1_L  3
#define Pin_D2_R  4
#define Pin_D2_L  5
#define Pin_E_R   6
#define Pin_E_L   7



void setup_motor()
{
  pinMode(Pin_D1_L, OUTPUT);
  pinMode(Pin_D2_L, OUTPUT);
  pinMode(Pin_E_L, OUTPUT);
  pinMode(Pin_D1_R, OUTPUT);
  pinMode(Pin_D2_R, OUTPUT);
  pinMode(Pin_E_R, OUTPUT);
}

void berhenti()
{
  analogWrite(Pin_E_L, 0);
  digitalWrite(Pin_D1_L, LOW);
  digitalWrite(Pin_D2_L, LOW);
  analogWrite(Pin_E_R, 0);
  digitalWrite(Pin_D1_R, LOW);
  digitalWrite(Pin_D2_R, LOW);
}


void majuGo(int kiri, int kanan)
{
  if (kiri < 0)
  {
    digitalWrite(Pin_D1_L, HIGH);
    digitalWrite(Pin_D2_L, LOW);
    kiri = -1 * kiri;
  }
  else
  {
    digitalWrite(Pin_D1_L, LOW);
    digitalWrite(Pin_D2_L, HIGH);
  }
  analogWrite(Pin_E_L, kiri);

  if (kanan < 0)
  {
    digitalWrite(Pin_D1_R, HIGH);
    digitalWrite(Pin_D2_R, LOW);
    kanan = -1 * kanan;
  }
  else
  {
    digitalWrite(Pin_D1_R, LOW);
    digitalWrite(Pin_D2_R, HIGH);
  }
  analogWrite(Pin_E_R, kanan);
}
