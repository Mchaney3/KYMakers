// Am I going insane? Am I so dumb that I can't control basic motors?

// Oh thank god the answer is no. Now why TF don't GPS work

const int IN1 = 4;
const int IN2 = 5;
const int IN3 = 6;
const int IN4 = 7;
const int ENA = 1;
const int ENB = 2;


void setup() {

  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);

}

void fullStop(){
    //control speed 
  analogWrite(ENA, 0);
  analogWrite(ENB, 0); 
//control direction 

//Motor A (Motor on right if looking from rear)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

//Motor B (Motor on left if looking from rear)  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void goForward(){
  //control speed 
  analogWrite(ENA, 64);
  analogWrite(ENB, 64); 
//control direction 

//Motor A (Motor on right if looking from rear)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

//Motor B (Motor on left if looking from rear)  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void goBackward(){
  //control speed 
  analogWrite(ENA, 64);
  analogWrite(ENB, 64); 
//control direction 

//Motor A (Motor on right if looking from rear)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

//Motor B (Motor on left if looking from rear)  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void trackRight(){
  //control speed 
  analogWrite(ENA, 64);
  analogWrite(ENB, 64); 
//control direction 

//Motor A (Motor on right if looking from rear)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

//Motor B (Motor on left if looking from rear)  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void trackLeft(){
  //control speed 
  analogWrite(ENA, 64);
  analogWrite(ENB, 64); 
//control direction 

//Motor A (Motor on right if looking from rear)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

//Motor B (Motor on left if looking from rear)  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void loop() {
  goForward();
  delay(2000);
  fullStop();
  delay(100);
  goBackward();
  delay(2000);
  fullStop();
  delay(100);
  trackLeft();
  delay(2000);
  fullStop();
  delay(100);
  trackRight();
  delay(2000);
  fullStop();
  delay(2000);
}
