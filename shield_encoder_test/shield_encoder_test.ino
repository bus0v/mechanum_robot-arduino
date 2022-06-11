
//FR,FL,BL,BR

// define pin lists
const int encA[] = {3, 19, 18, 2};
const int encB[] = {26, 15, 10, 5};


volatile int newPosition [] = {0,0,0,0};

long t0 = 0;
float e0 = 0;
float eInt = 0;


void setup(){
  Serial.begin(9600);
  for (int k = 0; k < 4; k++){
    pinMode(encA[k], INPUT);
    pinMode(encB[k], INPUT);

    //2.5 0.8 0.5 works with 5% accuracy
    attachInterrupt(digitalPinToInterrupt(encA[0]),readEncoder<0>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[1]),readEncoder<1>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[2]),readEncoder<2>,RISING);
    attachInterrupt(digitalPinToInterrupt(encA[3]),readEncoder<3>,RISING);

}}



void loop(){
  //read position
  int pos[4];
  noInterrupts();
  for(int k = 0; k < 4; k++){
     pos[k] = newPosition[k];
    }
  interrupts();

  // loop through the motors


  // make this into a loop
  Serial.print("FR FL    BL    BR ");
  Serial.println();
  for (int p = 0; p < 4; p++){
    Serial.print(newPosition[p]);
    Serial.print(" ");
  }
  Serial.println();
}

template <int j>
void readEncoder(){
  int b = digitalRead(encB[j]);
  if(b > 0){
    newPosition[j]++;
  }
  else{
    newPosition[j]--;
  }

}
