
void setup()
{
  Serial.begin(115200);
}

int get_value(){
  int zero_get = 0;
  
  
  for(int i = 0;i<150;i++){
      if(digitalRead(A5)==0){
          zero_get++;
        }    
    delay(1);

 }
  float temp = zero_get/150.0;
  
    Serial.println(temp);
  if(temp>0.28 && temp < 0.37){
      return 600;
    }
    else if(temp>0.17 && temp < 0.23){
      return 1500;
    }
   else return 0;
}

void loop()
{
  
 
  Serial.println(get_value());
    
  delay(100);
}
