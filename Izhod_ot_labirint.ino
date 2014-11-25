// motors
enum MotorData
{
Back,
Speed,
Forward
};

#define Kp 1
#define target 400

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

SenData lsen;

void setup()
{
  for(int i = 0; i < 3; i++)
  {
    pinMode(l_motor[i], OUTPUT);
    pinMode(r_motor[i], OUTPUT);
  }
  
  pinMode(l_sen, INPUT);
  pinMode(r_sen, INPUT);
  
  lsen = new SenData(l_sen);
}

unsigned byte do_calc(int sen_data)
{
  int error = target - sen_data;
  int result = 0;
  error *= Kp;
  result = map(error, -400, 600, 0, 255);
  return result;
}

void loop()
{
  lsen.getReading();
  
}
