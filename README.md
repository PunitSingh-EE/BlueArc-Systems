# BlueArc Systems
Smart Distribution Transformer Monitoring + Wire-Break Detection + Adaptive Street Lighting

**Hacksagon National Hardware Hackathon 2026 — Top 150 / ~5,000 entries**

## System Architecture
LoRa 433MHz mesh: A2 → A1 → Master (DT) → Receiving Station

## Hardware
ESP32, LoRa SX1278, ACS71020, INA260, MPU6050, WS2812, ST7789 TFT, SD, DS3231

## Features
- Wire-break detection < 1.1s with relay trip
- Adaptive streetlight: 0% day / 15% night saver / 100% on motion
- Full DT power quality: V, I, P, Q, S, PF, Hz via ACS71020
- SCADA TFT display + SD data logging + WiFi web dashboard
- 5-day field trial: 90,001 CSV records logged

## Team
L.D. College of Engineering, Ahmedabad
