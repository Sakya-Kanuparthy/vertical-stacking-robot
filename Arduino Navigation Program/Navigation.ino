#include <stdio.h>
#include <math.h>

#define cpr 164
#define PI 3.14159265358979

#define pwm1 2
#define pwm2 3
#define leftIR 7
#define rightIR 8
#define enc2a 9
#define enc1a 10
#define echoPin 12
#define trigPin 13
#define enc2b 14
#define enc1b 15
#define pwmCon 17
#define dirCon 18
#define dir2 20
#define dir1 21

// 2 = Right ; 1 = Left (Encoder Motor)

double waypoint[][2] = {{14, 58.7}, {16.6539, 65.6303}, {163.642, 170.401}, {165.6, 169.8} };
 
 /*
double waypoint[][2] = {{14, 58.7}, {34, 68}, {58,89}, {82, 108}, {97, 188}, {126, 138}, {165.6, 169.8}};

double waypoint[][2] = { {22.8119, 63.4049}, {41.0017, 68.437}, {56.4126, 74.2526}, {66.6065, 83.4261}, {79.3138, 91.735}, {90.0985, 98.4563}, {102.277, 110.535}, {108.189, 115.186}, {112.776, 125.625}, {131.691, 134.592}, {141.848, 142.618}, {156.039, 152.297}, {161.597, 162.924}, {166.822, 171.617} };
*/

#define size1  (sizeof(waypoint)/16)
#define size2 ((sizeof(waypoint)/16)-1)

double Dist[size1][2];
double Ang[size2];
double u[size2], r[size2];
double omega1[size2], omega2[size2];
int t[size2];
double pwmVal1[size2], pwmVal2[size2];
double pow1, pow2;
double direc1[size2], direc2[size2];

// ENCODER VARIABLES
int speedcontrol = 0;
volatile long encoderValue = 0;
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;
int rpm = 0;

int pos;

long duration; // variable for the duration of sound wave travel
int distance;
double defaultSpeed = 80;

void setup() {
  
  Serial.begin(9600);
  
  pinMode(enc2a, INPUT_PULLUP);
  pinMode(enc2b, INPUT);
  attachInterrupt(digitalPinToInterrupt(enc2a), updateEncoder, RISING);
  pinMode(pwm2, OUTPUT);
  pinMode(dir2, OUTPUT);
  
  pinMode(enc1b, INPUT);
  pinMode(enc1a, INPUT);
  attachInterrupt(digitalPinToInterrupt(enc1a), updateEncoder, RISING);
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  
  pinMode(pwmCon, OUTPUT);
  pinMode(dirCon, OUTPUT);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);

  previousMillis = millis();

}

void updateEncoder()  {
  // Increment value for each pulse from encoder
  encoderValue++;
}

void moveRobot(double val1, double wDir1, double val2, double wDir2) { 
  if(wDir1 > 0 && wDir2 > 0) {
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, HIGH);
  }
  else if(wDir1 < 0 && wDir2 > 0) {
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);
  }
  else if(wDir1 > 0 && wDir2 < 0) {
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, LOW);
  }
  else if(wDir1 < 0 && wDir2 < 0) {
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  }
  analogWrite(pwm1, val1);
  analogWrite(pwm2, val2);
}

void goForward() {
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, HIGH);
  analogWrite(pwm1, defaultSpeed);
  analogWrite(pwm2, defaultSpeed);
}

void goBackward() {
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  analogWrite(pwm1, defaultSpeed);
  analogWrite(pwm2, defaultSpeed);
}

void turnRight() {
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, LOW);
  analogWrite(pwm1, defaultSpeed);
  analogWrite(pwm2, defaultSpeed);
}

void turnLeft() {
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);
  analogWrite(pwm1, defaultSpeed);
  analogWrite(pwm2, defaultSpeed);
}

void Off() {
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
}

void conveyorOn() {
  digitalWrite(dirCon, LOW);
  analogWrite(pwmCon, 255);
}

void conveyorOff() {
  digitalWrite(dirCon, HIGH);
  analogWrite(pwmCon, 255);
}

int obstacleDetection() {
  
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor

  return distance;
}

// CALCULATING ANGLE BETWEEN TWO POINTS
double angleCalc(double arr[][2], int size) {
    
    // printf("\n");
    // printf("Running angleCalc\n");
    for(int a = 0; a < size; a++) {
    if(floor(10000000000*( (arr[a+1][a+1] - arr[a][a+1]) / (arr[a+1][a] - arr[a][a]) )) / 10000000000 == PI/2) {
        // printf("tanTheta = 90 deg");
    }
    else {
        Ang[a] = atan(floor(10000000000*( (arr[a+1][a+1] - arr[a][a+1]) / (arr[a+1][a] - arr[a][a]) )) / 10000000000);
        // printf("Ang between [%d] & [%d] = %f rad\n", a, a+1, Ang[a]);
  }
  }
  // printf("\n");
}

// CALCULATING DISTANCE IN X AND Y AXIS
double distCalc(double arr[][2], int size) {
    
    // printf("\n");
    // printf("Running distCalc\n");
    for(int i=0; i<((size1)-1); i++) {
        // printf("Running xDist Loop\n");
        int j = 0;
        Dist[i][j] = arr[i+1][j] - arr[i][j];
        // printf("xDist between arr[%d][%d] and arr[%d][%d] = %f cm\n", i+1,j,i,j,Dist[i][j]);
        }
    
    for(int k=0; k<((size1)-1); k++) {
        // printf("Running yDist Loop\n");
        int l = 1;
        Dist[k][l] = arr[k+1][l] - arr[k][l];
        // printf("yDist between arr[%d][%d] and arr[%d][%d] = %f cm\n", k+1,l,k,l,Dist[k][l]);
        }
    // printf("\n");
}

// ARRAY OF SEGMENTS OF TIME
int timeArray() {
   t[0] = 1;
   t[1] = 5;
   t[2] = 1;
}

// CALCULATION OF U AND R
double urCalc(double distances[][2], double angles[]) {
    
    // printf("\n");
    for(int s=0; s < size2; s++) {
        // printf("dist %f cm\ttime %d s\tang %f rad \n", distances[s][0], t[s], angles[s]);
    }
    for(int f=0; f < size2; f++) {
        u[f] = (distances[f][0]/(t[f]*cos(angles[f])));
        // printf("u[%d] = %f\n", f, u[f]);
        r[f] = angles[f] / t[f];
        // printf("r[%d] = %f\n", f, r[f]);
    }
    // printf("\n");
}

double omegaCalc(double u[], double r[]) {
    
    // printf("\n");
    for(int v = 0; v < size2; v++) {
        omega1[v] = (0.2 * u[v]) - (2.3 * r[v]);
        omega2[v] = (0.2 * u[v]) + (2.3 * r[v]);       
        // printf("omega1[%d] = %f \t omega2[%d] = %f\n", v, omega1[v], v, omega2[v]);
    }
    // printf("\n");
}

// PWM MAPPING
double pwmMap(double omega1[], double omega2[]) {
    
    // printf("\n");
    // RIGHT WHEEL
    // printf("Running omegaCalc for Right Wheel\n");
    for (int b = 0; b < size2; b++) {
       if (omega1[b] < 0) {
            // printf("%f i.e. omega1[%d] < 0\n", omega1[b], b);
            pow1 = pow((-1*omega1[b]), 0.9004008);
            pwmVal1[b] = 0.80 * 15.86293 * pow1;
    }
    
        else if (omega1[b] > 0) {
            // printf("%f i.e. omega1[%d] > 0\n", omega1[b], b);
            pow1 = pow(omega1[b], 0.9004008);
            pwmVal1[b] = 0.80 * 15.86293 * pow1;
    }
    
        else if (omega1[b] == 0) {
            // printf("%f i.e. omega1[%d] = 0\n", omega1[b], b);
            pwmVal1[b] = 0;
    } 
        // printf("omega1[%d] = %f \t pwmVal1[%d] = %f \n", b, omega1[b], b, pwmVal1[b]);
    }
    // printf("\n");
    // printf("Running omegaCalc for Left Wheel\n");
    // LEFT WHEEL
    for (int c = 0; c < size2; c++) {
       if (omega2[c] < 0) {
            // printf("%f i.e. omega2[%d] < 0\n", omega2[c], c);
            pow2 = pow((-1*omega2[c]), 0.9009783);
            pwmVal2[c] = 0.80 * 15.86951 * pow2;
    }
    
        else if (omega2[c] > 0) {
            // printf("%f i.e. omega2[%d] > 0\n", omega2[c], c);
            pow2 = pow(omega2[c], 0.9009783);
            pwmVal2[c] = 0.80 * 15.86951 * pow2;
    }
    
        else if (omega2[c] == 0) {
            // printf("%f i.e. omega2[%d] = 0\n", omega2[c], c);
            pwmVal2[c] = 0;
    } 
        // printf("omega2[%d] = %f \t pwmVal2[%d] = %f \n", c, omega2[c], c, pwmVal2[c]);
    }
    // printf("\n");
}    

// SET DIRECTION FOR WHEEL
double dirCalc(double u[], double r[]) {
    
    // printf("\n");
    // printf("Running dirCalc\n");
    for(int x = 0; x < size2; x++) {
        if (u[x] < 0) {
            direc1[x] = -1;
            // printf("for speed u[%d], dir = %f\n", x, direc1[x]);
        }
        else if (u[x] > 1) {
            direc1[x] = 1;
            // printf("for speed u[%d], dir = %f\n", x, direc1[x]);
        }
        else {
            direc1[x] = 1;
            // printf("for speed u[%d], dir = %f\n", x, direc1[x]);
        }
    }
    
    for(int y = 0; y < 3; y++) {
        if (r[y] < 0) {
            direc2[y] = -1;
            // printf("for speed r[%d], dir = %f\n", y, direc2[y]);
        }
        
        else if (r[y] > 1) {
            direc2[y] = 1;
            // printf("for speed r[%d], dir = %f\n", y, direc2[y]);
        }
        
        else {
            direc2[y] = 1;
            // printf("for speed r[%d], dir = %f\n", y, direc2[y]);
        }
    }
    // printf("\n");
}

// FUNCTION FOR NAVIGATION
double navigate() {
    
    distCalc(waypoint, size2);
    
    /* DISTANCE
    for(int q=0; q<(size1-1); q++) {
        for (int p=0; p<2; p++) {
        printf("q = %d\t", q);
        printf("p = %d\t", p);
        printf("Dist[%d][%d] = %f cm\n", q, p, Dist[q][p]);
    }}
    */
    
    angleCalc(waypoint, size2);
    
    /* ANGLE
    printf("Size Of Ang Array = %d\n", size2);
    for(int o = 0; o < size2; o++) {
        printf("%d Angle: %f rad %f deg\n", o+1, Ang[o], Ang[o]*57.2958);
    }
    */
    timeArray();
    
    urCalc(Dist, Ang);
    
    omegaCalc(u, r);
    
    pwmMap(omega1, omega2);
    
    dirCalc(omega1, omega2);
    
  Off();
  delay(1000);
  moveRobot(pwmVal1[0], direc1[0], pwmVal2[0], direc2[0]);
  Serial.print("move1: ");
  Serial.print(pwmVal1[0]); Serial.print("\t"); Serial.print(direc1[0]); Serial.print("\t"); Serial.print(pwmVal2[0]); Serial.print("\t"); Serial.print(direc2[0]); Serial.print("\t"); Serial.print("Time: "); Serial.print(t[0]);
  Serial.print("\n");
  delay(t[0]*1000);
  moveRobot(pwmVal1[1], direc1[1], pwmVal2[1], direc2[1]);
  Serial.print("move2: ");
  Serial.print(pwmVal1[1]); Serial.print("\t"); Serial.print(direc1[1]); Serial.print("\t"); Serial.print(pwmVal2[1]); Serial.print("\t"); Serial.print(direc2[1]); Serial.print("\t"); Serial.print("Time:"); Serial.print(t[1]);
  Serial.print("\n");
  delay(t[1]*1000);
  moveRobot(pwmVal1[2], direc1[2], pwmVal2[2], direc2[2]);
  Serial.print("move3: ");
  Serial.print(pwmVal1[2]); Serial.print("\t"); Serial.print(direc1[2]); Serial.print("\t"); Serial.print(pwmVal2[2]); Serial.print("\t"); Serial.print(direc2[2]); Serial.print("\t"); Serial.print("Time:"); Serial.print(t[2]);
  Serial.print("\n");
  delay(t[2]*1000);
  Off();
  delay(1000);
  
}

double lineFollow() {
  if(digitalRead(leftIR)==0 && digitalRead(rightIR)==0){
  goForward();
  Serial.print("Forward!\n");
  }
  else if(!digitalRead(leftIR)==0 && digitalRead(rightIR)==0){
  turnLeft();
  Serial.print("Left!\n");
  }
  else if(digitalRead(leftIR)==0 && !analogRead(rightIR)==0){
  turnRight();
  Serial.print("Right!\n");
  }
  else if(!digitalRead(leftIR)==0 && !digitalRead(rightIR)==0){
  Off();
  Serial.print("Off!\n");
  }

  }

void loop() {

  /*
  if (obstacleDetection() > 15) {
    // navigate();
    // Serial.print("YES!");
  }
  
  else if (obstacleDetection() < 15) {
    // Serial.print("NO!");  
  }
  */
  navigate();
  }
