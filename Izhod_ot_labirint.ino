// motors
enum MotorData
{
Back,
Speed,
Forward
};

#define Kp 0.15
#define Kd 0.8
#define target 600
#define interval 100
#define base_speed 150
#define min_speed 10
#define turn_delay 900
#define left_turn_dist 250
//#define SERDEBUG

int l_motor[] = {4, 6, 7};
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

void turn_right()
{
  analogWrite(l_motor[Speed], base_speed);
  analogWrite(r_motor[Speed], base_speed);
    
  digitalWrite(l_motor[Forward], HIGH);
  digitalWrite(r_motor[Forward], LOW);
  digitalWrite(l_motor[Back], LOW);
  digitalWrite(r_motor[Back], HIGH);
}

void turn_left()
{
  analogWrite(l_motor[Speed], base_speed);
  analogWrite(r_motor[Speed], base_speed);
  
  digitalWrite(l_motor[Forward], LOW);
  digitalWrite(r_motor[Forward], HIGH);
  digitalWrite(l_motor[Back], HIGH);
  digitalWrite(r_motor[Back], LOW);
}

void go_forward()
{
  analogWrite(l_motor[Speed], base_speed);
  analogWrite(r_motor[Speed], base_speed);
  
  digitalWrite(l_motor[Forward], HIGH);
  digitalWrite(r_motor[Forward], HIGH);
  digitalWrite(l_motor[Back], LOW);
  digitalWrite(r_motor[Back], LOW);
}

void go_back()
{
  analogWrite(l_motor[Speed], base_speed);
  analogWrite(r_motor[Speed], base_speed);
  
  digitalWrite(l_motor[Forward], LOW);
  digitalWrite(r_motor[Forward], LOW);
  digitalWrite(l_motor[Back], HIGH);
  digitalWrite(r_motor[Back], HIGH);
}

enum State
{
  Go_Forward,
  Turn_Left,
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
      if(Go_Forward == prev_state)
      {
        state = Turn_Recovery;
        lock_timeout = (millis()) + turn_delay/2;
      }
      else
      {
        state = Turn_Left;
        lock_timeout = (millis()) + turn_delay;
      }
      lock_state = 1;
    }
    else 
    {
      state = Go_Forward;
    }
  }
  else if(lock_timeout < (millis()))
  {
    if(Turn_Left == state)
    {
      state = Turn_Recovery;
      lock_timeout = (millis()) + 2*turn_delay;
    }
    else if(Turn_Right == state)
    {
      state = Turn_Recovery;
      lock_timeout = (millis()) + turn_delay;
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
    case Turn_Left:
    {
      turn_left();
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
#ifdef SERDEBUG
  Serial.println(state);
#endif
  determine_state();
  execute_state();
}
