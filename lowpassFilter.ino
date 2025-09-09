#include <Arduino.h>
#include <Wire.h>
#include "ms4525do.h"
#include <math.h>  // sqrtf, fabsf

/* MS4525DO 传感器对象 */
bfs::Ms4525do pres;

/* 参数 */
const float AIR_DENSITY   = 1.149f;   // kg/m^3
const float ALPHA         = 0.1f;     // 固定指数平滑系数（EMA）
const uint32_t SAMPLE_HZ  = 10;       // 采样频率 10 Hz
const uint32_t SEND_HZ    = 10;       // 发送频率 10 Hz

/* 定时相关（微秒） */
uint32_t sample_period_us = 1000000UL / SAMPLE_HZ;
uint32_t send_period_us   = 1000000UL / SEND_HZ;
uint32_t last_sample_us   = 0;
uint32_t last_send_us     = 0;

/* 状态量 */
float zero_offset        = 0.0f;  // 零偏
float filtered_pressure  = 0.0f;  // 滤波后的压差（Pa）
float latest_airspeed    = 0.0f;  // 最近一次计算的空速（m/s）

/* 空速计算：v = sqrt(2 * ΔP / ρ)，压差为负时取负号 */
float calculateAirspeed(float pressure_pa) {
  if (pressure_pa < 0.0f) return -sqrtf(2.0f * fabsf(pressure_pa) / AIR_DENSITY);
  return  sqrtf(2.0f * pressure_pa / AIR_DENSITY);
}

/* 零偏校准：静止条件下取平均 */
void calibrateZeroOffset() {
  const int num_samples = 1000;
  double sum = 0.0;               // 用 double 累加更稳
  for (int i = 0; i < num_samples; i++) {
    if (pres.Read()) sum += pres.pres_pa();
    delay(10);                    // 10 ms × 1000 ≈ 10 s
  }
  zero_offset = (float)(sum / num_samples);
}

/* CRC16 (Modbus-RTU，多项式 0xA001) */
uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

/* 发送帧：AA 55 | 04 | [float airspeed] | CRC(lo, hi) */
void send_airspeed_frame(float airspeed) {
  uint8_t frame[9];
  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = 4;                         // payload 长度
  memcpy(&frame[3], &airspeed, 4);      // 假设小端 + IEEE-754

  uint16_t crc = crc16_modbus(&frame[2], 1 + 4);  // 覆盖 len+payload
  frame[7] = uint8_t(crc & 0xFF);                 // CRC 低字节
  frame[8] = uint8_t((crc >> 8) & 0xFF);          // CRC 高字节

  Serial.write(frame, sizeof(frame));
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}                // 等待串口就绪（USB CDC）
  Wire.begin();
  Wire.setClock(400000);            // I2C 400 kHz

  // 传感器配置：地址 0x28，量程 -1~+1 PSI（按你的库用法）
  pres.Config(&Wire, 0x28, 1.0f, -1.0f);
  if (!pres.Begin()) {
    Serial.println("Error communicating with sensor");
    while (1) {}
  }

  calibrateZeroOffset();
  Serial.print("Zero offset (Pa): ");
  Serial.println(zero_offset, 6);

  // 初始化定时基准
  uint32_t now = micros();
  last_sample_us = now;
  last_send_us   = now;
}

void loop() {
  uint32_t now = micros();

  // 1) 到点采样 + 固定系数 EMA 平滑
  if ((uint32_t)(now - last_sample_us) >= sample_period_us) {
    last_sample_us += sample_period_us;     // 锁相式推进，长期更稳
    if (pres.Read()) {
      float raw_pressure = pres.pres_pa();
      float corrected    = raw_pressure - zero_offset;
      // 固定 α 的 EMA：不依赖 dt
      filtered_pressure  = ALPHA * corrected + (1.0f - ALPHA) * filtered_pressure;
      latest_airspeed    = calculateAirspeed(filtered_pressure);
    }
  }

  // 2) 到点发送（与采样同频 10 Hz；若不同频也可改 SEND_HZ）
  if ((uint32_t)(now - last_send_us) >= send_period_us) {
    last_send_us += send_period_us;         // 锁相式推进
    send_airspeed_frame(latest_airspeed);
  }

  // 无需 delay()，空转等待下个节拍
}
