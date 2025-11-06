#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

// WiFi credentials
const char* ssid = "Tran Van Tuan";
const char* password = "0353070860";

// Web server on port 80
WebServer server(80);

Adafruit_MPU6050 mpu;

#define EEPROM_SIZE 64

#define EN_PINA1 25
#define EN_PINA2 26
#define EN_PINB1 18
#define EN_PINB2 19

float Kp = 20.0;
float Ki = 0.050;
float Kd = 5.0;
bool FailSafe = false; // Fail-safe flag
int dir =0;
float eintegral = 0;

// Complementary filter variables
double accelAngle = 0;
double gyroAngle = 0;
double filteredAngle = 0;
double alpha = 0.98; // Complementary filter coefficient (higher = trust gyro more)
double angleFilterAlpha = 0.3; // Additional low-pass filter for angle (lower = faster response)
double prevFilteredAngle = 0;

// Timing variables
unsigned long currentTime = 0;
unsigned long lastTime = 0;
double deltaTime = 0;

// PID variables
float setpoint = 0.9;
float eprev = 0;
//PWM limit
int pwm_MAX = 210;
int pwm_MIN = 80;
// EEPROM save flag
bool savePending = false;


// ================== EEPROM Functions ==================
void saveParams() {
  EEPROM.put(0, Kp);
  EEPROM.put(sizeof(float), Ki);
  EEPROM.put(2 * sizeof(float), Kd);
  EEPROM.commit();
  Serial.println("Parameters saved to EEPROM");
}

void loadParams() {
  EEPROM.get(0, Kp);
  EEPROM.get(sizeof(float), Ki);
  EEPROM.get(2 * sizeof(float), Kd);

  // Set defaults if EEPROM is empty (NaN values)
  if (isnan(Kp)) Kp = 20.0;
  if (isnan(Ki)) Ki = 0.050;
  if (isnan(Kd)) Kd = 5.0;
  
  Serial.print("Loaded - Kp: "); Serial.print(Kp);
  Serial.print(", Ki: "); Serial.print(Ki);
  Serial.print(", Kd: "); Serial.println(Kd);
}

// ================== Web Server Functions ==================
String getHTML() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>PID Tuning</title>";
  html += "<style>";
  html += "body { font-family: Arial; margin: 20px; background: #f0f0f0; }";
  html += ".container { max-width: 600px; margin: auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }";
  html += "h1 { color: #333; text-align: center; }";
  html += ".param { margin: 20px 0; }";
  html += "label { display: block; margin-bottom: 5px; font-weight: bold; color: #555; }";
  html += ".slider-container { display: flex; align-items: center; gap: 10px; }";
  html += "input[type='range'] { flex: 1; height: 8px; border-radius: 5px; background: #ddd; outline: none; -webkit-appearance: none; }";
  html += "input[type='range']::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 20px; height: 20px; border-radius: 50%; background: #4CAF50; cursor: pointer; }";
  html += "input[type='range']::-moz-range-thumb { width: 20px; height: 20px; border-radius: 50%; background: #4CAF50; cursor: pointer; border: none; }";
  html += "input[type='number'] { width: 100px; padding: 8px; border: 2px solid #ddd; border-radius: 5px; font-size: 16px; text-align: center; }";
  html += ".btn { width: 100%; padding: 12px; margin: 10px 0; border: none; border-radius: 5px; font-size: 16px; cursor: pointer; }";
  html += ".btn-update { background: #4CAF50; color: white; }";
  html += ".btn-save { background: #2196F3; color: white; }";
  html += ".btn-load { background: #FF9800; color: white; }";
  html += ".current { background: #e7f3ff; padding: 15px; border-radius: 5px; margin: 20px 0; }";
  html += ".current h3 { margin-top: 0; color: #2196F3; }";
  html += ".value { font-size: 18px; color: #333; margin: 5px 0; }";
  html += "</style>";
  html += "<script>";
  html += "function updateValue(slider, display) {";
  html += "  document.getElementById(display).value = parseFloat(slider.value).toFixed(slider.step.includes('.') ? slider.step.split('.')[1].length : 0);";
  html += "}";
  html += "function updateSlider(input, slider) {";
  html += "  document.getElementById(slider).value = input.value;";
  html += "}";
  html += "</script>";
  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h1>PID Tuning Panel</h1>";
  
  html += "<div class='current'>";
  html += "<h3>Current Values</h3>";
  html += "<div class='value'>Kp: <strong>" + String(Kp, 2) + "</strong></div>";
  html += "<div class='value'>Ki: <strong>" + String(Ki, 1) + "</strong></div>";
  html += "<div class='value'>Kd: <strong>" + String(Kd, 2) + "</strong></div>";
  //html += "<div class='value'>Integral: <strong>" + String(eintegral, 3) + "</strong></div>";
  html += "</div>";
  
  html += "<form action='/update' method='GET'>";
  
  // Kp parameter with slider
  html += "<div class='param'>";
  html += "<label for='kp'>Kp (Proportional): 0 - 40</label>";
  html += "<div class='slider-container'>";
  html += "<input type='range' id='kp_slider' min='0' max='40' step='0.01' value='" + String(Kp, 2) + "' oninput=\"updateValue(this, 'kp')\">";
  html += "<input type='number' id='kp' name='kp' min='0' max='40' step='0.01' value='" + String(Kp, 2) + "' oninput=\"updateSlider(this, 'kp_slider')\">";
  html += "</div></div>";
  
  // Ki parameter with slider
  html += "<div class='param'>";
  html += "<label for='ki'>Ki (Integral): 0 - 80</label>";
  html += "<div class='slider-container'>";
  html += "<input type='range' id='ki_slider' min='0' max='80' step='0.1' value='" + String(Ki, 1) + "' oninput=\"updateValue(this, 'ki')\">";
  html += "<input type='number' id='ki' name='ki' min='0' max='80' step='0.1' value='" + String(Ki, 1) + "' oninput=\"updateSlider(this, 'ki_slider')\">";
  html += "</div></div>";
  
  // Kd parameter with slider
  html += "<div class='param'>";
  html += "<label for='kd'>Kd (Derivative): 0 - 5</label>";
  html += "<div class='slider-container'>";
  html += "<input type='range' id='kd_slider' min='0' max='5' step='0.01' value='" + String(Kd, 2) + "' oninput=\"updateValue(this, 'kd')\">";
  html += "<input type='number' id='kd' name='kd' min='0' max='5' step='0.01' value='" + String(Kd, 2) + "' oninput=\"updateSlider(this, 'kd_slider')\">";
  html += "</div></div>";
  
  html += "<button type='submit' class='btn btn-update'>Update Parameters</button>";
  html += "</form>";
  
  html += "<button onclick=\"location.href='/save'\" class='btn btn-save'>Save to EEPROM</button>";
  html += "<button onclick=\"location.href='/load'\" class='btn btn-load'>Load from EEPROM</button>";
  html += "<button onclick=\"location.reload()\" class='btn' style='background: #607D8B; color: white;'>Refresh</button>";
  
  html += "</div>";
  html += "<script>setTimeout(function(){ location.reload(); }, 5000);</script>";
  html += "</body></html>";
  
  return html;
}

void handleRoot() {
  server.send(200, "text/html", getHTML());
}

void handleUpdate() {
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
    Kp = server.arg("kp").toFloat();
    Ki = server.arg("ki").toFloat();
    Kd = server.arg("kd").toFloat();
    
    // Reset integral to prevent windup when changing parameters
    eintegral = 0;
    
    Serial.print("Updated - Kp: "); Serial.print(Kp);
    Serial.print(", Ki: "); Serial.print(Ki);
    Serial.print(", Kd: "); Serial.println(Kd);
    
    String response = getHTML();
    server.send(200, "text/html", response);
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

void handleSave() {
  savePending = true; // Set flag instead of saving immediately
  String response = getHTML();
  server.send(200, "text/html", response);
}

void handleLoad() {
  loadParams();
  eintegral = 0; // Reset integral
  String response = getHTML();
  server.send(200, "text/html", response);
}

 // ================== Hàm đọc MPU & điều khiển động cơ ==================

void readMPU6050() {
  // Update timing first - use micros() for higher frequency
  currentTime = micros();
  deltaTime = (currentTime - lastTime) / 1000000.0;  // Convert microseconds to seconds
  
  // Guard against very large deltaTime (first call or overflow)
  if (deltaTime > 0.1 || deltaTime <= 0) {
    deltaTime = 0.01; // 10ms default
  }
  
  lastTime = currentTime;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // Calculate pitch angle from accelerometer (in degrees)
  // Pitch is the rotation around Y-axis (forward/backward tilt) - THIS is what we need for balancing!
  accelAngle = atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  
  // Calculate pitch angle from gyroscope (integrate angular velocity around Y-axis)
  gyroAngle += g.gyro.y * deltaTime * 180.0 / PI;
  
  // Apply complementary filter for pitch angle
  double rawAngle = alpha * (prevFilteredAngle + g.gyro.y * deltaTime * 180.0 / PI) + 
                    (1.0 - alpha) * accelAngle;
  
  // Additional low-pass filter to reduce noise
  filteredAngle = angleFilterAlpha * prevFilteredAngle + (1.0 - angleFilterAlpha) * rawAngle;
  prevFilteredAngle = filteredAngle;
  
  // Constrain angle to reasonable limits
  filteredAngle = constrain(filteredAngle, -60, 60);
  static double prevAngleForSpeed = 0;

  // Debug timing and angles (uncomment if needed)
  // Serial.print("dt: "); Serial.print(deltaTime*1000); Serial.print("ms, ");
  // Serial.print("Accel: "); Serial.print(accelAngle); Serial.print(", ");
  // Serial.print("Filtered: "); Serial.println(filteredAngle);
  
}
void set_motor(int dir, int pwr){
  

    if (dir) {
        ///forward
        digitalWrite(EN_PINA1, false);
        digitalWrite(EN_PINA2, true);
        digitalWrite(EN_PINB1, true);
        digitalWrite(EN_PINB2, false);
        analogWrite(5,pwr);
        analogWrite(33,pwr);
    } else {
        ///backward
        digitalWrite(EN_PINA1, true);
        digitalWrite(EN_PINA2, false);
        digitalWrite(EN_PINB1, false);
        digitalWrite(EN_PINB2, true);
        int pid = pwr + 10;
        pid = constrain(pid, 70, 255);
        analogWrite(5,pid);
        analogWrite(33,pid);
    }  
}
void pid_calculate() {
  if(FailSafe) {
      set_motor(1, 0); // Stop motors in FailSafe
      eintegral = 0; // Reset integral
      Serial.println("FailSafe activated! Motors stopped.");
      return;
  }
    // Use the same deltaTime from readMPU6050() instead of separate timing
    float deltaT = deltaTime; // Already calculated in readMPU6050()
    // Inner loop PID for balancing
    float error = setpoint - filteredAngle;
    // if((error) > - 3 && (error) < 4 ) {
    //     error = 0;
    // }
    float derivative = Kd * ((error - eprev) / deltaT);
    derivative = constrain(derivative, -130, 130); // Limit derivative term
    eintegral += Ki * error * deltaT;
    eintegral = constrain(eintegral, -120, 120); // Limit integral term

    float output = (Kp * error) +  eintegral +  derivative; 
    int dir = (output >= 0) ? 1 : 0;
    int pwr = (int)fabs(output);       
    pwr = constrain(pwr, pwm_MIN, pwm_MAX);      

    //printf("MPU State: %.2f, Error: %.2f, Output: %d, I term: %.2f, D term: %.2f\n", filteredAngle, error, pwr, eintegral, derivative);
    //printf("DeltaT: %.4f s\n", deltaT);
    set_motor(dir, pwr);
    eprev = error;
    //eprev = corrected_angle_error;
    
}

void MPU_Setup() {
    digitalWrite(2,LOW);
    set_motor(1, 0);
    Serial.println("Initializing MPU...");
    // Initialize I2C and MPU6050
    Wire.begin();
    
    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      while (1) {
        delay(10);
      }
    }
    
    Serial.println("MPU6050 Found!");
    
    // Configure MPU6050 settings
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // Calibration delay
    Serial.println("Calibrating... Keep robot upright!");
    delay(3000);
    
    // Initialize variables
    filteredAngle = 0;
    prevFilteredAngle = 0;
    
    // Initialize timing with micros() for higher frequency
    digitalWrite(2, HIGH); // Turn on LED to indicate setup start
    lastTime = micros();

    Serial.println("MPU Setup complete!");
}

void setup() {
        
    pinMode(15, INPUT_PULLUP);
    pinMode(EN_PINA1, OUTPUT);
    pinMode(EN_PINA2, OUTPUT);
    pinMode(EN_PINB1, OUTPUT);
    pinMode(EN_PINB2, OUTPUT);
    pinMode(33,OUTPUT);
    pinMode(2,OUTPUT);// Led 
    digitalWrite(2,LOW);
    set_motor(1, 0);
    // ledcSetClockSource(LEDC_AUTO_CLK);
    // ledcAttachChannel(19, 5000, 8, 2); /// cau sau kenh 2
    // ledcAttachChannel(33, 5000, 8, 0);

  
  Serial.begin(38400); //Initializate Serial wo work well at 8MHz/16MHz
    
    delay(250);

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Connect to WiFi
  Serial.println("\nConnecting to WiFi...");
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP()); //IP address: 192.168.1.42
    
    // Setup web server routes
    server.on("/", handleRoot);
    server.on("/update", handleUpdate);
    server.on("/save", handleSave);
    server.on("/load", handleLoad);
    
    server.begin();
    Serial.println("Web server started!");
  } else {
    Serial.println("\nWiFi connection failed. Continuing without web server...");
  }
  
  // Load parameters from EEPROM
  loadParams();

  Serial.println("Initializing MPU...");
    // Initialize I2C and MPU6050
  Wire.begin();
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");
  
  // Configure MPU6050 settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Calibration delay
  Serial.println("Calibrating... Keep robot upright!");
  delay(3000);
  
  // Initialize variables
  filteredAngle = 0;
  prevFilteredAngle = 0;
  
  // Initialize timing with micros() for higher frequency
  digitalWrite(2, HIGH); // Turn on LED to indicate setup start
  lastTime = micros();

  Serial.println("Setup complete!");
}


void loop() {
    // Handle web server requests
    server.handleClient();
    
    // Save parameters when robot is not balancing (to avoid timing disruption)
    if (savePending) {
        saveParams();
        savePending = false;
    }
    MPU_Setup();
    while(digitalRead(15)== HIGH){
      server.handleClient(); 
      readMPU6050();
      eintegral = 0;
      // if(abs(filteredAngle) < 60) {
      //     FailSafe = false; // Reset FailSafe when within safe angle
      // } else {
      //   FailSafe = true;
      // }
        if (savePending) {
        saveParams();
        savePending = false;
    }
  }
      while(digitalRead(15)== LOW){
        server.handleClient(); // Handle web requests even during balancing
        readMPU6050();  
        if(abs(filteredAngle) < 60) {
          FailSafe = false; // Reset FailSafe when within safe angle
        } else FailSafe = true;
        pid_calculate();
    }
    
}

