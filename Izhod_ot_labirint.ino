// motors
enum MotorData
{
Back,
Speed,
Forward
};

#define Kp 0.24
#define Kd 0.7
#define target 550
#define interval 100
#define base_speed 165
#define min_speed 10
#define turn_delay 500
#define left_turn_dist 350
//#define SERDEBUG

int l_motor[] = {4,  6,  7};
int r_motor[] = {8, 10, 12};

// sensors
#define l_sen A1
#define r_sen A4
#define f_sen_pin 2

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
  pinMode(f_sen_pin, INPUT);
  
#ifdef SERDEBUG
  Serial.begin(9600);
#endif
}

int do_calc(int sen_data)
{
  static float prev_error;
  float error = target - sen_data;
  error *= Kp;
  error += (error - prev_error) * Kd;
  prev_error = error;
  return (int)error;
}

void adjustSpeed(int speed_adjustment)
{
  int l_spd = base_speed - speed_adjustment;
  int r_spd = base_speed + speed_adjustment;

  if (l_spd < min_speed)l_spd = min_speed;
  if (r_spd < min_speed)r_spd = min_speed;
  
  if (l_spd > 255)l_spd = 255;
  if (r_spd > 255)r_spd = 255;
  
  analogWrite(l_motor[Speed], l_spd);
  analogWrite(r_motor[Speed], r_spd);
  
#ifdef SERDEBUG
  Serial.print((base_speed - speed_adjustment));
  Serial.print(" ");
  Serial.println((base_speed + speed_adjustment));
#endif
}

// Do not use this function. Use defines instead
void motor_control(int l_speed, int r_speed, int lmf, int lmb, int rmf, int rmb)
{
  analogWrite(l_motor[Speed], l_speed);
  analogWrite(r_motor[Speed], r_speed);
    
  digitalWrite(l_motor[Forward], lmf);
  digitalWrite(r_motor[Forward], rmf);
  digitalWrite(l_motor[Back], lmb);
  digitalWrite(r_motor[Back], rmb);
}

#define turn_right() motor_control(base_speed, base_speed, HIGH, LOW, LOW, HIGH)
#define turn_left() motor_control(base_speed, base_speed, LOW, HIGH, HIGH, LOW)
#define go_forward() motor_control(base_speed, base_speed, HIGH, LOW, HIGH, LOW)
#define go_back() motor_control(base_speed, base_speed, LOW, HIGH, LOW, HIGH)

enum State
{
  Go_Forward,

  Turn_Right,
  Turn_Recovery,
  Go_Back,
  StateCount
};

int lock_state = 0;
unsigned long long lock_timeout = 0;
State state = StateCount;
State prev_state = StateCount;

void determine_state()
{
  int lsen_val = lsen.getReading();
  int fsen_val = digitalRead(f_sen_pin);
  
  if(0 == lock_state)
  {
    prev_state = state;
    if(Go_Back == prev_state)
    {
      state = Turn_Right;
      lock_timeout = (millis()) + turn_delay - 200;
      lock_state = 1;
    }
    else if(LOW == fsen_val)
    {
      state = Go_Back;
      lock_timeout = (millis()) + turn_delay/3;
      lock_state = 1;
    }
    else if(lsen_val < left_turn_dist)
    {
      state = Go_Forward;
    }
    else 
    {
      state = Go_Forward;
    }
  }
  else if(lock_timeout < (millis()))
  {
    if(Turn_Right == state)
    {
      state = Turn_Recovery;
      lock_timeout = (millis()) + 200;
    }
    else
    {
      lock_state = 0;
    }
  }
}

void execute_state()
{
  switch(state)
  {
    case Go_Forward:
    {
      int control;
      int senval;
      static unsigned long long time = 0;
      
      if(state != prev_state)
      {
        go_forward();
      }
      if(time < (millis()))
      {
        time = (millis()) + interval;
        senval = lsen.getReading();
        control = do_calc(senval);
        adjustSpeed(control);
      }
      break;
    }
    case Turn_Right:
    {
      turn_right();
      break;
    }
    case Turn_Recovery:
    {
      go_forward();
      break;
    }
    case Go_Back:
    {
      go_back();
      break;
    }
    default:
      break;
  }
}

int f_sen = 1;

void loop()
{
  lsen.setReading();
  determine_state();
  execute_state();
}
