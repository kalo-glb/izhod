// motors
enum MotorData
{
Back,
Speed,
Forward
};

#define Kp 0.1
#define target 400
#define interval 100
#define base_speed 100
#define min_speed 60
//#define SERDEBUG

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

SenData lsen(l_sen);

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
  
#ifdef SERDEBUG
  Serial.begin(9600);
#endif
}

int do_calc(int sen_data)
{
  float error = target - sen_data;
  error *= Kp;
  return (int)error;
}

int normaliseSpeed(int control_signal)
{
  static const int max_range = 
    ((255 - base_speed) > base_speed) ? 
      (base_speed) : 
      (255 - base_speed);
      
  if((control_signal + base_speed) > 255)
  {
    control_signal = max_range;
  }

  if(control_signal < -base_speed)
  {
    control_signal = -max_range;
  }
  
#ifdef SERDEBUG
  Serial.print(control_signal);
  Serial.print(" ");
#endif
  
  return control_signal;
}

void adjustSpeed(int speed_adjustment)
{
  int l_spd = base_speed - speed_adjustment;
  int r_spd = base_speed + speed_adjustment;

  //if (l_spd < min_speed)l_spd = min_speed;
  //if (r_spd < min_speed)r_spd = min_speed;
  
  analogWrite(l_motor[Speed], l_spd);
  analogWrite(r_motor[Speed], r_spd);
  
#ifdef SERDEBUG
  Serial.print((base_speed - speed_adjustment));
  Serial.print(" ");
  Serial.println((base_speed + speed_adjustment));
#endif
}

int control = 0;
int senval = 0;
int speed_delta = 0;
unsigned long long time = 0;

void loop()
{
  lsen.setReading();
  if(time < (millis()))
  {
    time = (millis()) + interval;
    senval = lsen.getReading();
    control = do_calc(senval);
    speed_delta = normaliseSpeed(control);
    adjustSpeed(speed_delta);
  }
}
