#include <EEPROM.h>

#define OFFSET 100

#define VR 2
#define TPS A0
#define COOLTEMP A1
#define NEUTIN 9
#define WHEEL_FR 3

#define CLK_HZ 16000000
#define PRE_SCL_RPM 64
#define EDGES 44

#define VOLTTOVAL(x) (((float) 1023 * (float) x) / (float) 5)

#define MAX_HALLMICROS 200000

volatile unsigned long rpmsum = 0;
volatile unsigned long rpmavgn = 0;
volatile unsigned long teethtime = 0;
volatile byte seq = 0;
volatile char started = 0;

int teeth = 0;
unsigned long startrot = 0;
unsigned long starttooth = 0;
int toothread = 0;

float TPSPOS = 0;
float TEMP = 0;
char NEUT = 0;
float WHEEL_SPD = 0;

int TPS_MIN = 0;
int addr_TPS_MIN = OFFSET;
int TPS_MAX = 1023;
int addr_TPS_MAX = addr_TPS_MIN + sizeof(TPS_MIN);

#define SENDDELAY 200
unsigned long LASTSEND = 0;

void setup()
{
  pinMode(VR, INPUT);
  //Pin change interrupts (D2, RPM)
  DDRD &= ~(1 << DDD2);
  PORTD |= (1 << PORTD2);
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18);

  pinMode(NEUT, INPUT_PULLUP);
  pinMode(WHEEL_FR, INPUT_PULLUP);
  Serial.begin(9600);

  EEPROM.get(addr_TPS_MIN, TPS_MIN);
  EEPROM.get(addr_TPS_MAX, TPS_MAX);

  //Timer 1 (RPM) setup
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  //TCCR1B |= (1 << CS11) | (1 << CS10); //64 prescale, but wait to start clock
  TIMSK1 = (1 << TOIE1); //only overflow

}

void loop()
{
  if (millis() - LASTSEND > SENDDELAY)
  {
    char rpm[6];
    char temp[4];
    char tps[4];
    char wheel[6];

    Serial.print("{R:");
    if (teethtime == 0 && rpmsum == 0)
    {
      sprintf(rpm, "%05d",  0);
    }
    else
    {
      sprintf(rpm, "%05d",  (int) (timetorpm(rpmsum) / (float) rpmavgn));
    }
    Serial.print(rpm);

    Serial.print(",T:");
    sprintf(temp, "%03d", (int) TEMP);
    Serial.print(temp);

    Serial.print(",P:");
    sprintf(tps, "%03d", (int) TPS);
    Serial.print(tps);

    Serial.print(",N:");
    Serial.print(NEUT);

    Serial.print(",W:");
    sprintf(wheel, "%05d", (int) WHEEL_SPD);
    Serial.print(wheel);

    Serial.println("}");

    LASTSEND = millis();

    if (Serial.read() == 's')
    {
      settps();
    }

    rpmavgn = 0;
  }

  int tpsraw = analogRead(TPS);
  TPSPOS = (((float) tpsraw - (float) TPS_MIN) / ((float) TPS_MAX - (float) TPS_MIN)) * 100;

  int tempraw = analogRead(COOLTEMP);
  TEMP = ((float) tempraw - 840.32) / -6.8687; //from excel

  NEUT = !digitalRead(NEUTIN);

  if (digitalRead(WHEEL_FR) == 0 && toothread == 0)
  {
    starttooth = micros();
    if (teeth == 0)
    {
      startrot = micros();
    }
    teeth++;
    teeth %= 30;
    toothread = 1;
  }
  else if (digitalRead(WHEEL_FR) ==  1 && toothread == 1)
  {
    WHEEL_SPD = calcrotv(micros() - startrot, teeth + 1);
    toothread = 0;
  }
  if (micros() - starttooth > MAX_HALLMICROS)
  {
    WHEEL_SPD = 0;
    teeth = 0;
  }

}

void settps()
{
  int tpsraw = 0;

  Serial.println("min");
  while (Serial.read() != 'd');
  tpsraw = analogRead(TPS);
  EEPROM.put(addr_TPS_MIN, tpsraw);
  TPS_MIN = tpsraw;

  Serial.println("max");
  while (Serial.read() != 'd');
  tpsraw = analogRead(TPS);
  EEPROM.put(addr_TPS_MAX, tpsraw);
  TPS_MAX = tpsraw;
}

float timetorpm(unsigned long x)
{
  if (x > 0)
  {
    return 1.0 / ((1.0 / ((float) CLK_HZ / (float) PRE_SCL_RPM)) * (float) x) * 60.0;
  }
  else
  {
    return 0;
  }
}

ISR(PCINT2_vect) //VR interrupt
{
  if (!started) //Now start it
  {
    TCCR1B |= (1 << CS11) | (1 << CS10);
    seq = 0;
    rpmavgn = 0;
    rpmsum = 0;
    teethtime = 0;
    TCNT1 = 0;
    started = 1;
  }
  else
  {
    seq++;
    seq %= EDGES;
    teethtime += TCNT1;
    TCNT1 = 0;
    if (seq == 0)
    {
      rpmavgn++;
      rpmsum += teethtime;
      teethtime = 0;
    }
  }
}

ISR(TIMER1_OVF_vect) //If timer1 overflows, engine is not running
{
  TCNT1 = 0;
  seq = 0;
  started = 0;
  teethtime = 0;
  rpmsum = 0;
  rpmavgn = 0;
}

float calcrotv(unsigned long timeMicros, int teeth)
{
  return ((teeth * 0.0544543) / timeMicros) * 1000000;
}
