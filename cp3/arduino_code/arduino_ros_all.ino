#include <ros.h>

#include <rosserial_arduino/my.h>

// Motor
boolean motor_start = false;

const byte L_pin_a = 2;   //for encoder pulse A motor left
const byte L_pin_b = 7;   //for encoder pulse B motor left
const byte L_pin_fwd = 4; //for H-bridge: run motor forward
const byte L_pin_bwd = 5; //for H-bridge: run motor backward
const byte L_pin_pwm = 6; //for H-bridge: motor speed


const byte R_pin_a = 3;   //for encoder pulse A motor right
const byte R_pin_b = 8;   //for encoder pulse B motor right
const byte R_pin_fwd = 9; //for H-bridge: run motor forward
const byte R_pin_bwd = 10; //for H-bridge: run motor backward
const byte R_pin_pwm = 11; //for H-bridge: motor speed


double set_speed = 0;
double L_set_speed = 0;
double R_set_speed = 0;

int L_encoder = 0;
int L_m_direction = 0;
double L_pv_speed = 0;
double L_e_speed = 0; //error of speed = set_speed - pv_speed
double L_e_speed_pre = 0;  //last error of speed
double L_e_speed_sum = 0;  //sum error of speed
double L_pwm_pulse = 0;     //this value is 0~255
double L_kp = 9.0;
double L_ki = 3.0;
double L_kd = 3.3;

int R_encoder = 0;
int R_m_direction = 0;
double R_pv_speed = 0;
double R_e_speed = 0; //error of speed = set_speed - pv_speed
double R_e_speed_pre = 0;  //last error of speed
double R_e_speed_sum = 0;  //sum error of speed
double R_pwm_pulse = 0;     //this value is 0~255
double R_kp = 8;
double R_ki = 2.6;
double R_kd = 2;
float Vl=0;
float Vr=0;

// ROS
ros::NodeHandle nh;

void callback(const rosserial_arduino::my &msg){
      
  Vr = (2.0*msg.linear+1.7*msg.angular)/2.0;
  Vl = (2.0*msg.linear-1.7*msg.angular)/2.0;
  
  
  if(Vr==0.0 && Vl==0.0){
    digitalWrite(L_pin_fwd,0);      //run motor run forward
    digitalWrite(L_pin_bwd,0);
    digitalWrite(R_pin_fwd,0);      //run motor run forward
    digitalWrite(R_pin_bwd,0);
    motor_start = false;
    L_set_speed=0;
    R_set_speed=0;
  }
  else{
    if (Vl>0){
      motor_start = true;
      digitalWrite(L_pin_fwd,1);      //run motor run forward
      digitalWrite(L_pin_bwd,0);
      L_set_speed = Vl;
    }else{
      motor_start = true;
      digitalWrite(L_pin_fwd,0);      //run motor run forward
      digitalWrite(L_pin_bwd,1);
      L_set_speed = abs(Vl);
    }
  
    if (Vr>0){
      motor_start = true;
      digitalWrite(R_pin_fwd,1);      //run motor run forward
      digitalWrite(R_pin_bwd,0);
      R_set_speed = Vr;
    }else{
      motor_start = true;
      digitalWrite(R_pin_fwd,0);      //run motor run forward
      digitalWrite(R_pin_bwd,1);
      R_set_speed = abs(Vr);
    }
  }
  
  double L_pv_speed = 0;
  double L_e_speed = 0; //error of speed = set_speed - pv_speed
  double L_e_speed_pre = 0;  //last error of speed
  double L_e_speed_sum = 0;  //sum error of speed
  double L_pwm_pulse = 0;     //this value is 0~255
  double R_pv_speed = 0;
  double R_e_speed = 0; //error of speed = set_speed - pv_speed
  double R_e_speed_pre = 0;  //last error of speed
  double R_e_speed_sum = 0;  //sum error of speed
  double R_pwm_pulse = 0;     //this value is 0~255
}
ros::Subscriber<rosserial_arduino::my> getter("get_num",&callback);

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(getter);

  // // left motor config
  pinMode(L_pin_a,INPUT_PULLUP);
  pinMode(L_pin_b,INPUT_PULLUP);
  pinMode(L_pin_fwd,OUTPUT);
  pinMode(L_pin_bwd,OUTPUT);
  pinMode(L_pin_pwm,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(L_pin_a), detect_L_a, RISING);
  
  // // right motor config
  pinMode(R_pin_a,INPUT_PULLUP);
  pinMode(R_pin_b,INPUT_PULLUP);
  pinMode(R_pin_fwd,OUTPUT);
  pinMode(R_pin_bwd,OUTPUT);
  pinMode(R_pin_pwm,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(R_pin_a), detect_R_a, RISING);
  
  //--------------------------timer setup
  timer_setup();
  
  // left motor config
  analogWrite(L_pin_pwm,0);   //stop motor
  digitalWrite(L_pin_fwd,0);  //stop motor
  digitalWrite(L_pin_bwd,0);  //stop motor
  
  // right motor config
  analogWrite(R_pin_pwm,0);   //stop motor
  digitalWrite(R_pin_fwd,0);  //stop motor
  digitalWrite(R_pin_bwd,0);  //stop motor
  L_encoder=0;
  R_encoder=0;
  pinMode(A0,INPUT);
  pinMode(0,INPUT);
  pinMode(12,INPUT_PULLUP);
  pinMode(13,INPUT_PULLUP);
}
int not_move=0;
void loop()
{
   if(digitalRead(0) == 0){
    motor_start = false;
      digitalWrite(L_pin_fwd,0);      //run motor run forward
      digitalWrite(L_pin_bwd,0);
      L_set_speed = 0;
      digitalWrite(R_pin_fwd,0);      //run motor run forward
      digitalWrite(R_pin_bwd,0);
      R_set_speed = 0;
      nh.spinOnce();

      nh.loginfo("stop");
    }
  if(digitalRead(13) == 0 || digitalRead(12) == 0){
    
    int min_value=1000;

    
      motor_start = true;
      digitalWrite(L_pin_fwd,0);      //run motor run forward
      digitalWrite(L_pin_bwd,1);
      L_set_speed = 5;
      digitalWrite(R_pin_fwd,0);      //run motor run forward
      digitalWrite(R_pin_bwd,1);
      R_set_speed = 5;
      
      delay(1000);

      
    motor_start = true;
    digitalWrite(L_pin_fwd,1);      //run motor run forward
    digitalWrite(L_pin_bwd,0);
    L_set_speed = 4.25;
    digitalWrite(R_pin_fwd,0);      //run motor run forward
    digitalWrite(R_pin_bwd,1);
    R_set_speed = 4.25;
    
     char log_msg[50];
     
    
    while(1){
      if(analogRead(A0)<min_value){
      min_value = analogRead(A0);
      not_move = 0;
//      sprintf(log_msg, "log min_value %d", (int)(analogRead(A0))); 
//      nh.loginfo(log_msg);
      }
      else{
      not_move++;
      }
    if(not_move>400){
      nh.loginfo("done");
        break;
      }
     
    nh.spinOnce();
    delay(10);
    }
    sprintf(log_msg, "min_value %d finish", (int)(min_value)); 
     nh.loginfo(log_msg);
   
//       char log_msg[20];
//       sprintf(log_msg, "min_value %d", (int)(min_value)); 
//       nh.loginfo(log_msg);
//       
   while(1){
//   sprintf(log_msg, "min_value %d", (int)(analogRead(A0))); 
//   nh.loginfo(log_msg);
   
     //if(analogRead(A0)<(min_value+5) && analogRead(A0) > (min_value-5)){
//     int a = analogRead(A0);
    sprintf(log_msg, "log choose value %d", (int)(analogRead(A0))); 
    nh.loginfo(log_msg);

    if(min_value>255) break;
    if(min_value<80) min_value = 120;
     if(analogRead(A0)<=(min_value)){
        
        motor_start = false;
        digitalWrite(L_pin_fwd,0);      //run motor run forward
        digitalWrite(L_pin_bwd,0);
        L_set_speed = 0;
        digitalWrite(R_pin_fwd,0);      //run motor run forward
        digitalWrite(R_pin_bwd,0);
        R_set_speed = 0;
        
        sprintf(log_msg, "now min value %d", (int)(analogRead(A0))); 
      nh.loginfo(log_msg);
        //nh.loginfo("finish");
        break;
     }else{
      motor_start = true;
      digitalWrite(L_pin_fwd,1);      //run motor run forward
      digitalWrite(L_pin_bwd,0);
      L_set_speed = 4.25;
      digitalWrite(R_pin_fwd,0);      //run motor run forward
      digitalWrite(R_pin_bwd,1);
      R_set_speed = 4.25;
      }
     nh.spinOnce();
   }
         delay(1000);

//     while(1){
//      char log_msg[20];
//    sprintf(log_msg, "L: %d", (int)digitalRead(8)); 
//    nh.loginfo(log_msg);
//    nh.spinOnce();
    
        motor_start = true;
        digitalWrite(L_pin_fwd,1);      //run motor run forward
        digitalWrite(L_pin_bwd,0);
        L_set_speed = 10;
        digitalWrite(R_pin_fwd,1);      //run motor run forward
        digitalWrite(R_pin_bwd,0);
        R_set_speed = 10;
          //nh.loginfo("why2");

        nh.spinOnce();
        delay(500);
//    }
  }
 
  
    
  nh.spinOnce();
  delay(100);
}

void timer_setup(void){
  //--------------------------timer setup
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    OCR1A = 12500; // 16MHz / 256 / 5Hz -> 5Hz
    
    TCCR1B |= (1 << WGM12);    // 256 prescaler 
    TCCR1B |= (1 << CS12);   // enable timer overflow interrupt
    TIMSK1 |= (1 << OCIE1A);

    interrupts();             // enable all interrupts
    //--------------------------timer setup
}

void detect_L_a() {
  L_encoder+=1; //increasing encoder at new pulse    
}
void detect_R_a() {
  R_encoder+=1; //increasing encoder at new pulse
//  R_m_direction = digitalRead(R_pin_b); //read direction of motor
    
}
int a_start = 0;
ISR(TIMER1_COMPA_vect)        // interrupt service routine - tick every 0.1sec
{
  
  
  L_pv_speed = 6.5*3.14*(L_encoder/120.0/16.0)/0.2;  //calculate motor speed, unit is rpm
  R_pv_speed = 6.5*3.14*(R_encoder/120.0/16.0)/0.2;  //calculate motor speed, unit is rpm
  //print out speed
//  Serial.print(R_pv_speed);
//  Serial.print(",");
//  Serial.println(L_pv_speed);         //Print speed (rpm) value to Visual Studio
  
  //print out encoder
//   Serial.print("encoder");
//   Serial.println(encoder);         //Print speed (rpm) value to Visual Studio
  L_encoder=0;
  R_encoder=0;

  //PID program
  if (motor_start){
    L_e_speed = L_set_speed - L_pv_speed;
    L_pwm_pulse = L_e_speed*L_kp + L_e_speed_sum*L_ki + (L_e_speed - L_e_speed_pre)*L_kd;
    L_e_speed_pre = L_e_speed;  //save last (previous) error
    L_e_speed_sum += L_e_speed; //sum of error
    if (L_e_speed_sum >4000) L_e_speed_sum = 4000;
    if (L_e_speed_sum <-4000) L_e_speed_sum = -4000;
  }
  else{
    L_e_speed = 0;
    L_e_speed_pre = 0;
    L_e_speed_sum = 0;
    L_pwm_pulse = 0;
  }
  if (motor_start){
    R_e_speed = R_set_speed - R_pv_speed;
    R_pwm_pulse = R_e_speed*R_kp + R_e_speed_sum*R_ki + (R_e_speed - R_e_speed_pre)*R_kd;
    R_e_speed_pre = R_e_speed;  //save last (previous) error
    R_e_speed_sum += R_e_speed; //sum of error
    if (R_e_speed_sum >4000) R_e_speed_sum = 4000;
    if (R_e_speed_sum <-4000) R_e_speed_sum = -4000;
  }
  else{
    R_e_speed = 0;
    R_e_speed_pre = 0;
    R_e_speed_sum = 0;
    R_pwm_pulse = 0;
  }
  
//  Serial.println(L_pwm_pulse);
  
  //update new speed
  if (L_pwm_pulse <255 & L_pwm_pulse >0){
    analogWrite(L_pin_pwm,L_pwm_pulse);  //set motor speed  
    
  }
  else{
    if (L_pwm_pulse>255){
      analogWrite(L_pin_pwm,255);
    }
    else{
      analogWrite(L_pin_pwm,0);
    }
  }
  //update new speed
  if (R_pwm_pulse <255 & R_pwm_pulse >0){
    analogWrite(R_pin_pwm,R_pwm_pulse);  //set motor speed  
    
  }
  else{
    if (R_pwm_pulse>255){
      analogWrite(R_pin_pwm,255);
    }
    else{
      analogWrite(R_pin_pwm,0);
    }
  }
}
