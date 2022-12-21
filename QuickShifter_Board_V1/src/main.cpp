#include <Arduino.h>
#include <EEPROM.h>
#include "HX711_Arduino_Library\src\HX711.h"
#include "HX711_Arduino_Library\src\HX711.cpp"
// #include <Wire.h>

#define HX711_SCL 7
#define HX711_SDA 8

#define MCU_RESET_PIN (1 << 0)
#define MCU_RESET_PIN_SET PORTC
#define MCU_RESET(state) state ? MCU_RESET_PIN_SET &= ~MCU_RESET_PIN : MCU_RESET_PIN_SET |= MCU_RESET_PIN

#define BLUETOOTH_STATE_PIN (1 << 4)
#define BLUETOOTH_STATE_PIN_INPUT_SET PIND
#define GET_BLUETOOTH_STATE (BLUETOOTH_STATE_PIN_INPUT_SET & BLUETOOTH_STATE_PIN)

#define MIN_CUT_TIME 10
#define MAX_CUT_TIME 1000
#define CUT_PIN (1 << 3)
#define CUT_PIN_PWM_REGISTER OCR2B
#define CUT_PIN_SET PORTD
// #define CUT(state) state ? CUT_PIN_SET |= CUT_PIN : CUT_PIN_SET &= ~CUT_PIN
#define CUT_PWM(duty) CUT_PIN_PWM_REGISTER = duty

#define MONITOR_CYCLE_TIME 10
#define MONITOR_OUTPUT_TIMER TIMSK1
#define MONITOR_OUTPUT(state) state ? MONITOR_OUTPUT_TIMER |= (1 << OCIE1A) : MONITOR_OUTPUT_TIMER &= ~(1 << OCIE1A)

#define SENSOR_RST_TIME 5000
#define CHECK_DELAY 10

#define MAX_speed_accelerate 3
#define ISR_delaytime 10

volatile uint64_t ISR_delaytimer;
volatile uint32_t RPM;

HX711 Gear_Scale;

int32_t S, T, G, Sensor, angle, G_force;

const uint32_t S_gain_map_lab[] = {2000, 2500, 3000, 3500, 4000, 5000, 6000, 8000, 10000, 14000};
const uint32_t T_gain_map_lab[] = {0, 5, 10, 20, 30, 40, 50, 60, 80, 100};

const uint8_t SETDATA_SIZE = 6 + 2 + 2 + 10 + 10 + 5;

volatile static uint8_t cut_enable = 1;
volatile static uint8_t gear_scale_invert = 0;
volatile static uint8_t gear_scale_sensitivity = 100;
volatile static uint8_t engine_start_delay = 80;
volatile static uint8_t cut_time = 80;
volatile static uint8_t cut_smooth_time = 30;

volatile static uint8_t stable_delay[] = {50, 50};
volatile static uint8_t cut_valve[] = {100, 100};
volatile static uint8_t S_gain_map[] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
volatile static uint8_t T_gain_map[] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
volatile static uint8_t G_gain_map[] = {120, 110, 100, 100, 100};

volatile uint32_t engine_start_cheak_num = 0;
uint64_t engine_start_cheak_timer = 0;

volatile bool serial_busy_flag = false;
ISR(TIMER1_COMPA_vect)
{
  if (serial_busy_flag)
    return;
  serial_busy_flag = true;
  sei();
  PORTB ^= (1 << 5);
  String buffer = "m" + (String)S + "," + (String)T + "," + (String)G + "," + (String)Sensor + "," + (String)angle + "," + (String)G_force;
  Serial.println(buffer);
  serial_busy_flag = false;
}

void RPM_ISR()
{
  static volatile uint64_t RPM_time;
  static volatile uint32_t RPM_array[3];
  static volatile byte RPM_add;
  static volatile bool RPM_flag;
  if (millis() < ISR_delaytimer)
    return;
  if (!RPM_flag)
  {
    RPM_time = micros();
    RPM_flag = true;
    return;
  }
  volatile uint64_t RPM_now = 120000000 / (micros() - RPM_time);
  ISR_delaytimer = millis() + ISR_delaytime;
  RPM_flag = false;
  if (RPM_now > 14000 && RPM_now < 10)
    return;
  RPM_array[RPM_add] = RPM_now;
  if (++RPM_add > 2)
    RPM_add = 0;
  RPM = (RPM_array[0] + RPM_array[1] + RPM_array[2]) / 3;
}
void Cut(uint32_t time)
{
  if (time == 0)
    return;
  Serial.println(time);
  time = constrain(time, MIN_CUT_TIME, MAX_CUT_TIME);
  CUT_PWM(255);
  uint64_t timer = millis() + time;
  while (millis() < timer)
    ;

  uint64_t smooth_time = cut_smooth_time * 1000;
  timer = micros();
  while (micros() < timer + smooth_time + 100)
    CUT_PWM(255 - uint32_t(constrain(pow(float((micros() - timer)) / float(smooth_time), 2.0) * 255.0, 0, 255)));
  // CUT_PWM(constrain(255 - uint32_t(pow(float((micros() - timer)) / float(smooth_time), 1) * 255.0), 0, 255));

  CUT_PWM(0);
}
void Get_S_T()
{
  static uint64_t last_S_timer, last_T_timer;
  static uint32_t last_S, last_T, last_T_s;

  static float accelerate;

  if ((millis() + ISR_delaytime) - ISR_delaytimer >= 1000)
  {
    S = 0;
    return;
  }

  S = min((MAX_speed_accelerate * (millis() - last_S_timer)) + last_S, RPM);
  last_S = S;
  last_S_timer = millis();

  if (millis() - last_T_timer > 100)
  {
    T = pow(max(S - 1500, 0) * 2, 0.3);
    accelerate = (float(S - last_T_s) / float(millis() - last_T_timer)) * 1000.0;
    // Serial.println(accelerate);
    if (accelerate < -50)
      T = max(last_T - 3, 0);
    else
      T = constrain(min(T + (pow(max(accelerate, 0) / 20, 1.6) * 0.2), last_T + 2), 0, 100);

    last_T_s = S;
    last_T = T;
    last_T_timer = millis();
  }
}

void Get_Sensor()
{
  static uint64_t sensor_val_reset_timer;
  Sensor = constrain((Gear_Scale.get_units(2)), -200, 200);

  if (abs(Sensor) > 20)
  {
    if (sensor_val_reset_timer == 0)
      sensor_val_reset_timer = millis();
    if (millis() - sensor_val_reset_timer >= SENSOR_RST_TIME && sensor_val_reset_timer != 0)
    {
      Gear_Scale.tare();
      sensor_val_reset_timer = 0;
      Sensor = 0;
    }
  }
  else
    sensor_val_reset_timer = 0;
}

void Get_G_To_Cut()
{
  static bool cut_flag;
  static bool N_flag;
  static uint8_t check_state;
  static uint64_t check_timer;
  G = constrain(G, 0, 6);

  uint8_t shift_state = (Sensor >= 0) ? 0 : 1;

  if (abs(Sensor) <= cut_valve[shift_state] * 0.4)
  {
    check_state = 0;
    cut_flag = false;
  }
  if (cut_flag)
    return;

  switch (check_state)
  {
  case 0:
    if (abs(Sensor) >= cut_valve[shift_state] * 0.6)
    {
      if (shift_state == 0)
        if (G == 1 && S < 2000)
          N_flag = true;

      check_state = 1;
      check_timer = millis() + CHECK_DELAY;
    }
    break;
  case 1:
    if (abs(Sensor) >= cut_valve[shift_state] && millis() > check_timer)
    {
      cut_flag = true;
      uint8_t cut_G = min(G, 5);
      if (shift_state == 1)
      {
        G = max(G - 1, 1);
        return;
      }
      else
      {
        G = min(G + 1, 6);
        if (N_flag)
        {
          G = 0;
          N_flag = false;
          return;
        }
        if (cut_G == 0)
        {
          G = 2;
          cut_G = 1;
        }
        if (S < S_gain_map_lab[0] || S > S_gain_map_lab[9])
          return;
        delay(stable_delay[0]);
        uint8_t time_map[4] = {0, 0, 0, 0};
        bool time_map_flag = false;
        for (uint8_t i = 0; i < 10 - 1 && !time_map_flag; i++)
        {
          if (S >= S_gain_map_lab[i] && S <= S_gain_map_lab[i + 1])
          {
            time_map[0] = i;
            time_map[1] = i + 1;
            if (time_map[1] > 9 || time_map[1] == 0)
              time_map[1] = time_map[0];
            time_map_flag = true;
          }
        }
        time_map_flag = false;
        for (uint8_t i = 0; i < 10 - 1 && !time_map_flag; i++)
        {
          if (T >= T_gain_map_lab[i] && T <= T_gain_map_lab[i + 1])
          {
            time_map[2] = i;
            time_map[3] = i + 1;
            if (time_map[3] > 9 || time_map[3] == 0)
              time_map[3] = time_map[2];
            time_map_flag = true;
          }
        }
        float S_gain = (((float(S) - S_gain_map_lab[time_map[0]]) / (S_gain_map_lab[time_map[1]] - S_gain_map_lab[time_map[0]]) * (S_gain_map[time_map[1]] - S_gain_map[time_map[0]])) + S_gain_map[time_map[0]]) / 100.0;
        float T_gain = (((float(T) - T_gain_map_lab[time_map[2]]) / (T_gain_map_lab[time_map[3]] - T_gain_map_lab[time_map[2]]) * (T_gain_map[time_map[3]] - T_gain_map[time_map[2]])) + T_gain_map[time_map[2]]) / 100.0;
        float G_gain = float(G_gain_map[cut_G - 1]) / 100.0;
        uint32_t final_cut_time = uint32_t(float(cut_time) * S_gain * T_gain * G_gain);
        if (cut_enable == 0)
          final_cut_time = 0;
        serial_busy_flag = true;
        Cut(final_cut_time);
        if (GET_BLUETOOTH_STATE)
        {
          String buffer = "c" + (String)S + "," + (String)T + "," + (String)cut_G + "," + (String)cut_time + "," + (String)final_cut_time + "," + (String)S_gain + "," + (String)T_gain + "," + (String)G_gain;
          Serial.println(buffer);
        }
        delay(stable_delay[1]);
        serial_busy_flag = false;
      }
      break;
    }
  }
}

bool SetData_Receive(uint8_t *data_array)
{
  uint64_t time_out_timer = millis() + 5000;
  char val = '0';
  String data = "";
  uint8_t data_add = 0;

  while (millis() < time_out_timer)
  {
    if (Serial.available() > 0)
    {
      val = Serial.read();
      if (val == '!')
        return true;
      if (val == ',')
      {
        data_array[data_add++] = constrain(data.toInt(), 0, 254);
        data = "";
      }
      else
        data += val;
      time_out_timer = millis() + 100;
    }
  }
  return false;
}

void Get_SetData_Array(uint8_t *data)
{
  uint8_t data_add = 0;
  data[data_add++] = cut_enable;
  data[data_add++] = gear_scale_invert;
  data[data_add++] = gear_scale_sensitivity;
  data[data_add++] = engine_start_delay;
  data[data_add++] = cut_time;
  data[data_add++] = cut_smooth_time;

  for (uint8_t i = 0; i < sizeof(stable_delay); i++)
    data[data_add++] = stable_delay[i];
  for (uint8_t i = 0; i < sizeof(cut_valve); i++)
    data[data_add++] = cut_valve[i];
  for (uint8_t i = 0; i < sizeof(S_gain_map); i++)
    data[data_add++] = S_gain_map[i];
  for (uint8_t i = 0; i < sizeof(T_gain_map); i++)
    data[data_add++] = T_gain_map[i];
  for (uint8_t i = 0; i < sizeof(G_gain_map); i++)
    data[data_add++] = G_gain_map[i];
}
void Set_SetData_Array(uint8_t *data)
{
  uint8_t data_add = 0;
  cut_enable = data[data_add++];
  gear_scale_invert = data[data_add++];
  gear_scale_sensitivity = data[data_add++];
  engine_start_delay = data[data_add++];
  cut_time = data[data_add++];
  cut_smooth_time = data[data_add++];

  for (uint8_t i = 0; i < sizeof(stable_delay); i++)
    stable_delay[i] = data[data_add++];
  for (uint8_t i = 0; i < sizeof(cut_valve); i++)
    cut_valve[i] = data[data_add++];
  for (uint8_t i = 0; i < sizeof(S_gain_map); i++)
    S_gain_map[i] = data[data_add++];
  for (uint8_t i = 0; i < sizeof(T_gain_map); i++)
    T_gain_map[i] = data[data_add++];
  for (uint8_t i = 0; i < sizeof(G_gain_map); i++)
    G_gain_map[i] = data[data_add++];
}
void EEPROM_Write(uint8_t *data, uint8_t size)
{
  if (data == NULL)
  {
    for (uint8_t i; i < size; i++)
      if (EEPROM.read(i) != 0)
        EEPROM.write(i, 0);
    delay(50);
    return;
  }
  for (uint8_t i = 0; i < size; i++)
    if (data[i] != EEPROM.read(i + 1))
      EEPROM.write(i + 1, data[i]);
}
bool EEPROM_Read(uint8_t *data, uint8_t size)
{
  if (EEPROM.read(0) == 0)
  {
    EEPROM.write(0, 1);
    return false;
  }
  for (uint8_t i = 0; i < size; i++)
    data[i] = EEPROM.read(i + 1);
  return true;
}

void Serial_Connect()
{
  static String buffer = "";
  static char val = '\n';
  static uint64_t mcu_reset_timer = millis();

  if (!GET_BLUETOOTH_STATE)
  {
    MONITOR_OUTPUT(false);
    return;
  }

  if (Serial.available() > 0)
  {
    PORTB ^= (1 << 5);
    mcu_reset_timer = millis() + 100;
    val = Serial.read();
    buffer += val;
    if (val == '\n' && buffer.length() > 0)
    {
      String read_data = buffer;
      read_data.remove(0, 1);
      read_data.replace("\n", "");
      read_data.replace("\r", "");

      switch (buffer[0])
      {
      case 'm':
        MONITOR_OUTPUT(read_data.toInt() == 1);
        break;
      case 'l':
        if (read_data.toInt() == 1)
        {
          serial_busy_flag = true;
          String string_data = "l";
          for (uint8_t i = 0; i < 10; i++)
            string_data += String(S_gain_map_lab[i]) + ",";
          for (uint8_t i = 0; i < 10; i++)
            string_data += String(T_gain_map_lab[i]) + ",";
          Serial.println(string_data);
          serial_busy_flag = false;
        }
        break;
      case 'r':
        if (read_data.toInt() == 1)
        {
          serial_busy_flag = true;
          uint8_t data_array[SETDATA_SIZE];
          Get_SetData_Array(data_array);
          String string_data = "r";
          for (uint8_t i = 0; i < SETDATA_SIZE; i++)
            string_data += String(data_array[i]) + ",";
          Serial.println(string_data);
          serial_busy_flag = false;
        }
        break;
      case 'w':
        uint8_t data_array[SETDATA_SIZE];
        switch (read_data.toInt())
        {
        case 0:
          serial_busy_flag = true;
          if (SetData_Receive(data_array))
            Set_SetData_Array(data_array);
          else
            Serial.println("ERR");
          serial_busy_flag = false;
          break;
        case 1:
          Get_SetData_Array(data_array);
          EEPROM_Write(data_array, SETDATA_SIZE);
          break;
        case 99:
          EEPROM_Write(NULL, SETDATA_SIZE);
          MCU_RESET(true);
          break;
        }
        break;
      case 'e':
        cut_enable = (read_data.toInt() != 0) ? 1 : 0;
        break;
      case 'c':
        Cut(read_data.toInt());
        break;
      case 'k':
        MCU_RESET(true);
        break;
      case 's':
        S = read_data.toInt();
        break;
      case 't':
        T = read_data.toInt();
        break;
      case 'g':
        if (read_data.toInt() == 1)
          G++;
        else
          G--;
        break;
      case 'h':
        Sensor = read_data.toInt();
        break;
      }
      buffer = "";
    }
  }
  else
  {
    if (val != '\n' && mcu_reset_timer < millis())
      MCU_RESET(true);
  }
}

void setup()
{
  DDRC |= MCU_RESET_PIN;
  MCU_RESET(false);

  engine_start_cheak_timer = millis() + 500;
  attachInterrupt(
      0, []
      { engine_start_cheak_num++; },
      FALLING);

  Serial.begin(115200);

  uint8_t data_array[SETDATA_SIZE];
  if (!EEPROM_Read(data_array, SETDATA_SIZE))
  {
    Get_SetData_Array(data_array);
    EEPROM_Write(data_array, SETDATA_SIZE);
  }
  else
    Set_SetData_Array(data_array);

  Gear_Scale.begin(HX711_SDA, HX711_SCL);
  Gear_Scale.set_scale(Gear_Scale.get_units(10) / (gear_scale_sensitivity * 20 * (gear_scale_invert == 1 ? -1 : 1)));
  Gear_Scale.tare();

  cli(); // 禁止中斷
  TCCR1A = 0;
  TCCR1B = (1 << WGM12);                // CTC mode; Clear Timer on Compare
  TCCR1B |= (1 << CS10) | (1 << CS11);  // Prescaler == 64
  OCR1A = MONITOR_CYCLE_TIME * 250 - 1; // TOP count for CTC, 與 prescaler 有關
  TCNT1 = 0;                            // counter 歸零
  sei();                                // 允許中斷
  MONITOR_OUTPUT(false);

  DDRD |= CUT_PIN;                   // set CUT_PIN output
  TCCR2A = _BV(COM2B1) | _BV(WGM20); // set CUT_PIN_PWM
  TCCR2B = _BV(CS20);                // set CUT_PIN_PWM frequency

  while (millis() < engine_start_cheak_timer)
    delay(1);
  if (GET_BLUETOOTH_STATE)
    Serial.println("OK");
  if (engine_start_cheak_num <= 3)
  {
    CUT_PWM(255);
    while (cut_enable == 1)
    {
      Serial_Connect();
      Get_Sensor();
      if (engine_start_cheak_num >= 5)
      {
        delay(engine_start_delay * 10);
        break;
      }
    }
  }
  CUT_PWM(0);
  attachInterrupt(0, RPM_ISR, FALLING);
}
void loop()
{
  Get_S_T();
  Get_Sensor();
  Get_G_To_Cut();
  Serial_Connect();
}