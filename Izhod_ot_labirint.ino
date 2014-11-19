int l_sen = A1;
int r_sen = A4;

int l_dist = 0;
int r_dist = 0;

void setup()
{
  pinMode(l_sen, INPUT);
  pinMode(r_sen, INPUT);
  Serial.begin(9600);
} 

void loop()
{
  l_dist = analogRead(l_sen);
  r_dist = analogRead(r_sen);
  Serial.print(l_dist);
  Serial.print(" ");
  Serial.println(r_dist);
  delay(400);
}
