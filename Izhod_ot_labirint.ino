// motors
enum MotorData
{
Back,
Speed,
Forward
};

#define Kp 0.5
#define target 400
#define interval 20
#define base_speed 100

int l_motor[] = {4, 6, 7};
int r_motor[] = {8, 10, 12};

// sensors
#define l_sen A1
#define r_sen A4

int l_dist = 0;
int r_dist = 0;

#define MAX_SEN_SAMPLES 10

class SenData
{
  private:
    int sen_data[MAX_SEN_SAMPLES];
    int index;
    int sensor;
    
  public:
    SenData(int sensor)
    {
      this->index = 0;
      this->sensor = sensor;
    }
    
    void setReading()
    {
      int data = analogRead(this->sensor);
      this->sen_data[this->index++] = data;
      this->index %= MAX_SEN_SAMPLES;
    }
    
    int getReading()
    {
      int sum = 0;
      for(int i = 0; i < MAX_SEN_SAMPLES; i++)
      {
        sum += this->sen_data[i];
      }
      
      sum /= MAX_SEN_SAMPLES;
      return sum;
    }
};

SenData *lsen;

void setup()
{
  // motors
  for(int i = 0; i < 3; i++)
  {
    pinMode(l_motor[i], OUTPUT);
    pinMode(r_motor[i], OUTPUT);
  }
  
  digitalWrite(l_motor[Forward], HIGH);
  digitalWrite(r_motor[Forward], HIGH);
  digitalWrite(l_motor[Back], LOW);
  digitalWrite(r_motor[Back], LOW);
  
  // sensors
  pinMode(l_sen, INPUT);
  pinMode(r_sen, INPUT);
  
  lsen = new SenData(l_sen);
}

int do_calc(int sen_data)
{
  float error = target - sen_data;
  error *= Kp;
  return (int)error;
}

int normaliseSpeed(int control_signal)
{
  if((control_signal + base_speed) > 255)
  {
    control_signal = 255 - base_speed;
  }
  else if((control_signal + base_speed) < -255)
  {
    control_signal = -255 + base_speed;
  }
  
  return control_signal;
}

void adjustSpeed(int speed_adjustment)
{
  analogWrite(l_motor[Speed], (base_speed + speed_adjustment));
  analogWrite(r_motor[Speed], (base_speed - speed_adjustment));
}

int control = 0;
int senval = 0;
int speed_delta = 0;
unsigned long long time = 0;

void loop()
{
  lsen->getReading();
  if(time < (millis()))
  {
    time = (millis()) + interval;
    senval = lsen->getReading();
    control = do_calc(senval);
    speed_delta = normaliseSpeed(control);
    adjustSpeed(speed_delta);
  }
}
