enum MotorData
{
  Back,
  Speed,
  Forward
};

int l_motor[] = {7, 6, 4};
int r_motor[] = {8, 10, 12};

void setup()
{
  pinMode(l_motor[Back], OUTPUT);
  pinMode(l_motor[Speed], OUTPUT);
  pinMode(l_motor[Forward], OUTPUT);
  pinMode(r_motor[Back], OUTPUT);
  pinMode(r_motor[Speed], OUTPUT);
  pinMode(r_motor[Forward], OUTPUT);
  
  analogWrite(l_motor[Speed], 100);
  analogWrite(r_motor[Speed], 100);
  
  digitalWrite(l_motor[Back], LOW);
  digitalWrite(l_motor[Forward], LOW);
  digitalWrite(r_motor[Back], LOW);
  digitalWrite(r_motor[Forward], LOW);
}

void loop()
{
  digitalWrite(l_motor[Back], HIGH);
  digitalWrite(r_motor[Back], HIGH);
  delay(3000);
  digitalWrite(l_motor[Back], LOW);
  digitalWrite(r_motor[Back], LOW);
  delay(300);
  digitalWrite(l_motor[Forward], HIGH);
  digitalWrite(r_motor[Forward], HIGH);
  delay(3000);
  digitalWrite(l_motor[Forward], LOW);
  digitalWrite(r_motor[Forward], LOW);
  delay(300);
}
