#include <Wire.h>

// =================================================================
// 1. KHU VỰC THÔNG SỐ CƠ BẢN (CHỈNH SỬA TẠI ĐÂY)
// =================================================================
// CÁC CHÂN ĐIỀU KHIỂN
#define STEP_1 3
#define DIR_1 4
#define STEP_2 6
#define DIR_2 5
#define En_R    9
#define En_L   10

// --- BƯỚC 1: ĐIỀN ĐIỂM CÂN BẰNG THỰC TẾ VÀO ĐÂY ---
float angle_offset = 7.2; 
// ---------------------------------------------------

// HỆ SỐ PID KÉP (CASCADE PID)
// Vòng trong (Inner Loop): Ổn định góc nghiêng (Phản ứng nhanh)
#define KP_ANGLE 9.0      
#define KI_ANGLE 0.05    
#define KD_ANGLE 0.4     

// Vòng ngoài (Outer Loop): Ổn định vận tốc/vị trí (Phản ứng chậm hơn)
#define KP_VELOCITY 0.16   
#define KI_VELOCITY 0.04  

// GIỚI HẠN VẬT LÝ 
#define MAX_CONTROL_OUTPUT 250  // Tốc độ xung tối đa gửi tới động cơ
#define MAX_ACCEL 20            // Gia tốc tối đa chống giật motor
#define MAX_TARGET_ANGLE 8.0    // Góc nghiêng tối đa vòng ngoài được phép yêu cầu (độ)

#define VELOCITY_ITERM_MAX 5.0  // Giới hạn tích phân chống trôi vòng vận tốc
#define ANGLE_ITERM_MAX 50.0    // Giới hạn tích phân chống quá tải vòng góc

// Cấu hình MPU6050
const int MPU_addr = 0x68;
float gyroY_offset = 0;
const float alpha = 0.98; // Hệ số bộ lọc bù

// =================================================================
// 2. BIẾN TOÀN CỤC
// =================================================================
long timer_old; 
float dt; 
float angle_adjusted = 0; 
float angle_adjusted_Old = 0; 

// Các biến phục vụ PID Cascade
float target_angle = 0;    // Đầu ra của vòng ngoài, đầu vào của vòng trong
float control_output = 0;  // Đầu ra của vòng trong (tốc độ motor)

float angle_error_sum = 0;
float velocity_error_sum = 0;

// Tốc độ và xung motor
volatile int16_t speed_M1 = 0, speed_M2 = 0; 
volatile int8_t dir_M1, dir_M2; 
volatile int32_t count_M1 = 0, count_M2 = 0;

// =================================================================
// 3. THUẬT TOÁN PID CASCADE CHỐNG WINDUP & D-KICK
// =================================================================

// VÒNG NGOÀI: Ổn định vận tốc (Trả về Góc nghiêng mục tiêu)
float openOuterLoop_VelocityPID(float DT, int16_t current_velocity, int16_t target_velocity) {
  float error = target_velocity - current_velocity;
  
  // 1. Tích phân chống Windup bằng cách giới hạn cứng miền tích lũy (Saturate Limiting)
  velocity_error_sum += error * DT;
  velocity_error_sum = constrain(velocity_error_sum, -VELOCITY_ITERM_MAX, VELOCITY_ITERM_MAX);
  
  // Tính toán góc nghiêng mục tiêu
  float target_ang = (KP_VELOCITY * error) + (KI_VELOCITY * velocity_error_sum);
  
  // Giới hạn góc nghiêng mục tiêu để xe không nghiêng quá sâu dẫn đến ngã
  return constrain(target_ang, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE);
}

// VÒNG TRONG: Ổn định góc nghiêng (Trả về Tốc độ motor)
float openInnerLoop_AnglePID(float DT, float current_angle, float target_ang) {
  float error = target_ang - current_angle;
  
  // --- TRÁNH DERIVATIVE KICK ---
  // Thay vì lấy vi phân từ lỗi (d_error/dt), ta lấy vi phân trực tiếp từ sự thay đổi của cảm biến (d_input/dt).
  // Công thức: d(Error)/dt = d(Target)/dt - d(Input)/dt. Vì Target thay đổi đột ngột tạo ra cú sốc, ta coi d(Target)/dt = 0.
  float error_derivative = -(current_angle - angle_adjusted_Old) / DT;
  
  // --- TRÁNH INTEGRAL WINDUP (Thuật toán Clamping nâng cao) ---
  // Chỉ cho phép tích lũy I khi hệ thống chưa bị bão hòa đầu ra (chưa chạm MAX_CONTROL_OUTPUT)
  // Hoặc khi có xu hướng kéo lỗi về (đầu ra và lỗi ngược dấu nhau)
  bool saturated = (control_output >= MAX_CONTROL_OUTPUT && error > 0) || 
                   (control_output <= -MAX_CONTROL_OUTPUT && error < 0);
                   
  if (!saturated) {
    angle_error_sum += error * DT;
    angle_error_sum = constrain(angle_error_sum, -ANGLE_ITERM_MAX, ANGLE_ITERM_MAX); // Thêm một tầng bảo vệ cứng cho I-term
  }
  
  // Tính toán đầu ra PID
  float output = (KP_ANGLE * error) + (KI_ANGLE * angle_error_sum) + (KD_ANGLE * error_derivative);
  
  return constrain(output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
}

// =================================================================
// 4. ĐỌC MPU6050 THÔ VÀ BỘ LỌC BÙ (LẬT NGƯỢC TRỤC Z)
// =================================================================
void updateMPU6050(float DT) {
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  
  
  AcX = Wire.read() << 8 | Wire.read();  
  AcY = Wire.read() << 8 | Wire.read();  
  AcZ = Wire.read() << 8 | Wire.read();  
  Wire.read() << 8 | Wire.read(); 
  GyX = Wire.read() << 8 | Wire.read();  
  GyY = Wire.read() << 8 | Wire.read();  
  GyZ = Wire.read() << 8 | Wire.read();  

  // Xử lý lật ngược trục Z
  AcZ = -AcZ; 
  AcX = -AcX;
  
  // Gia tốc và vận tốc góc theo đúng hệ tọa độ đã đảo bụng
  float angle_acc_pitch = atan2((float)AcX, (float)AcZ) * 57.2957795 + angle_offset; 
  float gyroY_rate = -((float)GyY - gyroY_offset) / 131.0; 

  angle_adjusted = alpha * (angle_adjusted + gyroY_rate * DT) + (1.0 - alpha) * angle_acc_pitch;
}

void calibrateMPU6050() {
  long gyroY_sum = 0;
  const int samples = 200;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x45); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 2, true);
    gyroY_sum += (Wire.read() << 8 | Wire.read());
    delay(5);
  }
  gyroY_offset = (float)gyroY_sum / samples;
}

// =================================================================
// 5. TRÌNH PHỤC VỤ NGẮT TIMER 1 (PHÁT XUNG TẦN SỐ CỐ ĐỊNH 25kHz)
// =================================================================
ISR(TIMER1_COMPA_vect) {
  int16_t s1 = speed_M1;
  if (s1 != 0) {
    if (s1 > 0) count_M1 += s1;
    else count_M1 -= s1;
    if (count_M1 >= 500) {
      digitalWrite(STEP_1, HIGH);
      delayMicroseconds(1);
      digitalWrite(STEP_1, LOW);
      count_M1 -= 500;
    }
  }

  int16_t s2 = speed_M2;
  if (s2 != 0) {
    if (s2 > 0) count_M2 += s2;
    else count_M2 -= s2;
    if (count_M2 >= 500) {
      digitalWrite(STEP_2, HIGH);
      delayMicroseconds(1);
      digitalWrite(STEP_2, LOW);
      count_M2 -= 500;
    }
  }
}

// =================================================================
// 6. CÁC HÀM ĐIỀU KHIỂN TỐC ĐỘ MOTOR
// =================================================================
void setMotorSpeedM1(int16_t tspeed) {
  noInterrupts();
  int16_t current_speed = speed_M1;
  interrupts();

  int16_t next_speed;
  if ((current_speed - tspeed) > MAX_ACCEL) next_speed = current_speed - MAX_ACCEL;
  else if ((current_speed - tspeed) < -MAX_ACCEL) next_speed = current_speed + MAX_ACCEL;
  else next_speed = tspeed;

  noInterrupts(); 
  speed_M1 = next_speed;
  if (speed_M1 == 0) { dir_M1 = 0; }
  else if (speed_M1 > 0) { dir_M1 = 1; digitalWrite(DIR_1, 1); } 
  else { dir_M1 = -1; digitalWrite(DIR_1, 0); }                 
  interrupts();
}

void setMotorSpeedM2(int16_t tspeed) {
  noInterrupts();
  int16_t current_speed = speed_M2;
  interrupts();

  int16_t next_speed;
  if ((current_speed - tspeed) > MAX_ACCEL) next_speed = current_speed - MAX_ACCEL;
  else if ((current_speed - tspeed) < -MAX_ACCEL) next_speed = current_speed + MAX_ACCEL;
  else next_speed = tspeed;

  noInterrupts();
  speed_M2 = next_speed;
  if (speed_M2 == 0) { dir_M2 = 0; }
  else if (speed_M2 > 0) { dir_M2 = 1; digitalWrite(DIR_2, 0); } 
  else { dir_M2 = -1; digitalWrite(DIR_2, 1); }                 
  interrupts();
}

void setEnableMotor(bool enable) {
  if (enable) {
    digitalWrite(En_R, LOW); 
    digitalWrite(En_L, LOW); 
  } else {
    digitalWrite(En_R, HIGH);
    digitalWrite(En_L, HIGH);       
  }
}

// =================================================================
// 7. SETUP
// =================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  
  pinMode(En_R, OUTPUT); pinMode(En_L, OUTPUT);
  pinMode(STEP_1, OUTPUT); pinMode(DIR_1, OUTPUT);
  pinMode(STEP_2, OUTPUT); pinMode(DIR_2, OUTPUT);
  
  digitalWrite(STEP_1, 0); digitalWrite(DIR_1, 0); 
  digitalWrite(STEP_2, 0); digitalWrite(DIR_2, 0);

  setEnableMotor(false); 
  delay(100);
  Serial.println(F("\n\n--- KHOI DONG: XE CAN BANG ---"));
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); 
  Wire.write(0);     
  if(Wire.endTransmission() != 0) {
    Serial.println(F("LOI I2C MPU6050!"));
    while(1);
  }
  
  delay(500);
  Serial.println(F("Dang lay offset Gyro..."));
  calibrateMPU6050();
  Serial.println(F("Da san sang!"));

  // KHỞI TẠO TIMER 1 (25kHz)
  noInterrupts();
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  OCR1A = 639;             
  TCCR1B |= (1 << WGM12);  
  TCCR1B |= (1 << CS10);   
  TIMSK1 |= (1 << OCIE1A); 
  interrupts();

  delay(200);
  timer_old = micros();
}

// =================================================================
// 8. VÒNG LẶP CHÍNH (XỬ LÝ PID VÒNG KÉP)
// =================================================================
void loop() {
  long timer_value = micros();
  
  if (timer_value - timer_old >= 10000) { // Chu kỳ chuẩn 10ms (100Hz)
    dt = (timer_value - timer_old) / 1000000.0;
    timer_old = timer_value;

    // 1. Cập nhật góc nghiêng từ bộ lọc bù (Xử lý lật trục Z)
    updateMPU6050(dt);

    // Đọc tốc độ hiện tại một cách an toàn từ luồng ngắt
    noInterrupts();
    int16_t current_speed = (speed_M1 + speed_M2) / 2;
    interrupts();

    // --- KIỂM TRA ĐIỀU KIỆN GÓC AN TOÀN ---
    if ((angle_adjusted > -40) && (angle_adjusted < 40)) {
      setEnableMotor(true); 

      // 2. VÒNG NGOÀI (PID VẬN TỐC): Đọc tốc độ hiện tại, tính ra góc mục tiêu (target_angle)
      target_angle = openOuterLoop_VelocityPID(dt, current_speed, 0);

      // 3. VÒNG TRONG (PID GÓC NGHIÊNG): Ép xe bám sát theo góc mục tiêu vừa tính.
      control_output = openInnerLoop_AnglePID(dt, angle_adjusted, target_angle);

      // Điều khiển chiều quay động cơ theo phần cứng của bạn (-control_output)
      setMotorSpeedM1(-control_output); 
      setMotorSpeedM2(-control_output); 
    } 
    // XE NGÃ -> RESET TOÀN BỘ BỘ TÍCH PHÂN ĐỂ TRÁNH QUÁ TẢI KHI DỰNG XE DẬY
    else {
      setEnableMotor(false); 
      setMotorSpeedM1(0); 
      setMotorSpeedM2(0); 
      velocity_error_sum = 0;
      angle_error_sum = 0;
      control_output = 0;
    }
    
    // Lưu lại giá trị góc để phục vụ tính Vi phân chống D-kick vòng tiếp theo
    angle_adjusted_Old = angle_adjusted;
  }

  // --- IN THÔNG SỐ MONITOR (100ms) ---
  static long last_print_time = 0;
  if (millis() - last_print_time > 100) {
    Serial.print(F("Pitch: "));         Serial.print(angle_adjusted, 1);
    Serial.print(F(" | Target_Ang: ")); Serial.print(target_angle, 1); 
    Serial.print(F(" | Out_Speed: "));  Serial.println(control_output, 0);
    last_print_time = millis();
  }
}