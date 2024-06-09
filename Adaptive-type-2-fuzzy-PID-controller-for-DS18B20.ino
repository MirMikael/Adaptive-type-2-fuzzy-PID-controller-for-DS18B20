// Created by Mir Mikael
// For quastions: MirMikael.github.io

#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

#define SENSOR_PIN  27
OneWire oneWire(SENSOR_PIN);
DallasTemperature DS18B20(&oneWire);
const int bit1 = 16;
const int MAX_DUTY_CYCLE = (int)(pow(2, bit1) - 1);
const float eta = 0.1;

float c[3][11] = {0};

float x[3] = {0};
float currentTime = 0;
float previousTime = 0;
float elapsedTime = 0;
float error = 0;
float lastError = 0;
float input, output, set, out = 0;

void error1(float setPoint, float inp) {
  currentTime = millis() * 0.001 ;
  elapsedTime = (currentTime - previousTime) + 0.00001;
  x[0] = setPoint - inp; // determine error
  x[1] += x[0] * elapsedTime ; // compute integral
  x[2]  = (x[0] - lastError) / elapsedTime ; // compute derivative
  lastError = x[0]; //remember current error
  previousTime = currentTime;
}

float MFu[3][11] = {0};
float MFl[3][11] = {0};
float au[11] = {0}, al[11] = {0};
//float szu = 0.0001, szl = 0.0001;
float do_dwr[11] = {0}, do_dwl[11] = {0};
float a = -2, b = 0;
int z = 0, i = 0, uc = 0, u_old = 0, uc2 = 0;
float oup = 0, olp = 0, o_kp = 0, wu_kp[11] = {15}, wl_kp[11] = {1};
float oui = 0, oli = 0, o_ki = 0, wu_ki[11] = {0.15}, wl_ki[11] = {1};
float oud = 0, old = 0, o_kd = 0, wu_kd[11] = {3}, wl_kd[11] = {1};

float dy_du = 0, y = 0, y_old = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  set = 16.21;
  DS18B20.begin();
  ledcSetup(0, 50, bit1);
  ledcAttachPin(2, 0);
  ledcSetup(2, 10, bit1);
  ledcAttachPin(33, 0);
  ledcWrite(0, 0);
  ledcWrite(2, 0);
  for (int ii = 0; ii < 3; ii++) {
    for (int i = 1; i < 11; i++)  {
      c[ii][i] = a;
      a += 1;
    }
    a = -2;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  DS18B20.requestTemperatures();
  y_old = y;
  y = DS18B20.getTempCByIndex(0);
  error1(set, y);
  Serial.println(currentTime);

  float szu = 0.0001, szl = 0.0001;
  float zu[11] = {1.0f}, zl[11] = {1.0f};

  for (int i = 0; i < 11; i++)  {
    for (int ii = 0; ii < 3; ii++) {
      //    exp(-(x - c). ^ 2. / sigmau. ^ 2);
      a = pow((x[ii] - c[i]), 2);
      MFu[ii][i] = exp(-a / 1); //pow(sigmau, 2)=1
      MFl[ii][i] = exp(-a / 0.01); //pow(sigmal, 2)=0.01
      zu[i] *= MFu[ii][i];
      zl[i] *= MFl[ii][i];
    }
    szu += zu[i];
    szl += zl[i];
  }

for (int i = 0; i < 11; i++)  {
    do_dwr[i] = zu[i]/ (szu + 0.0001);
    do_dwl[i] = zl[i] / (szl + 0.0001);

    oup += do_dwr[i] * wu_kp[i];
    olp += do_dwl[i] * wl_kp[i];

    oui += do_dwr[i] * wu_ki[i];
    oli += do_dwl[i] * wl_ki[i];

    oud += do_dwr[i] * wu_kd[i];
    old += do_dwl[i] * wl_kd[i];
}

  o_kp = (oup + olp) / 2;
  o_ki = (oui + oli) / 2;
  o_kd = (oud + old) / 2;


  u_old = uc;
  uc = o_kp * x[0] + o_ki * x[1] + o_kd * x[2];
  uc2 = -1 * uc;
  if (uc2 < 0) uc2 = 0;
  if (uc2 >= MAX_DUTY_CYCLE) uc2 = MAX_DUTY_CYCLE;

  ledcWrite(0, uc2);
  ledcWrite(2, uc2);
  dy_du = (y - y_old) / ((uc - u_old) + 0.0001);

  for (int i = 0; i < 11; i++) {

    a = eta * x[0] * dy_du;
    b = a * do_dwl[i];
    a *= do_dwr[i];

    wu_kp[i] += a * x[0];
    wl_kp[i] += b * x[0];
    wu_ki[i] += a * x[1];
    wl_ki[i] += b * x[1];
    wu_kd[i] += a * x[2];
    wl_kd[i] += b * x[2];
  }
}
