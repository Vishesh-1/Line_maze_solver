int sensor0 = A4; //leftmost sensor, pin A4
int sensor1 = A3;
int sensor2 = A2;
int sensor3 = A1;
int sensor4 = A0;// rightmost sensor, pin A0

int mata0; // mata means “eye”
int mata1; 
int mata2; 
int mata3; 
int mata4;



int R1 = 4; //forward or reverse, pin 4
int R2 = 5;
int onR = 10; //magnitude of the power for the motor
int onL = 11; //magnitude of the power for the motor
int L1 = 2; //forward or reverse, pin 7
int L2 = 3;
int data;
unsigned char s[5]; //array for sensors

int pathlength; //variable to record the total of path length
int readpath; //variable to call the path record
char path[99]; //array for path record

int threshold = 990; //threshold between black/white

//led to show what the robot is recording (L/R/S/U)
//int BLUEled = 13;
//int GREENled = 12;
//int YELLOWled = 11;

void strongright()
{
  analogWrite(onL, 127);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  analogWrite(onR, 53);//44
  digitalWrite(R1, HIGH);//to keep the robot on the line
  digitalWrite(R2, LOW);
}

void midright2()
{
  analogWrite(onL, 127);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  analogWrite(onR, 53);
  digitalWrite(R1, LOW);//to keep the robot on the line
  digitalWrite(R2, HIGH);
  
}

void midright()
{
  analogWrite(onL, 127);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  analogWrite(onR, 63); // 48
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void softright()
{
  analogWrite(onL, 127);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  analogWrite(onR, 112);// 88
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void moveforward()
{
  analogWrite(onL, 110);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  analogWrite(onR, 110);
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void softleft()
{
  analogWrite(onL, 112);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  analogWrite(onR, 127);
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void midleft()
{
  analogWrite(onL, 63);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  analogWrite(onR, 127);
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void midleft2()
{
 analogWrite(onL, 53);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  analogWrite(onR, 127);
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void strongleft()
{
  analogWrite(onL, 53);
  digitalWrite(L1, HIGH);
  digitalWrite(L2, LOW);
  analogWrite(onR, 127);
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void turnright()
{
  analogWrite(onL, 110);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH);
  analogWrite(onR, 110);
  digitalWrite(R1, HIGH);
  digitalWrite(R2, LOW);
}

void turnleft()
{
  analogWrite(onL, 110);
  digitalWrite(L1, HIGH);
  digitalWrite(L2, LOW);
  analogWrite(onR, 110);
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH);
}

void righttillstraight()
{
  turnright();
  delay(200); //to be free from the line if there is a straight intersection (exit 2)
  readsensor();
  while (s[2]==0)
  {
    turnright();
    readsensor();
  }
}

void lefttillstraight()
{
  turnleft();
  delay(200); //to be free from the line if there is a straight intersection (exit 2)
  readsensor();
  while (s[2]==0)
  {
    turnleft();
    readsensor();
  }
}

void turnaround()
{
  righttillstraight();
}

void stop()
{  
  analogWrite(onL, 0);
  digitalWrite(L1, LOW);
  digitalWrite(L2, LOW);
  analogWrite(onR, 0);
  digitalWrite(R1, LOW);
  digitalWrite(R2, LOW);
}

void lilmoveforward()
{
  moveforward();
  delay(100); // adjust based on your batteries power, new one would be stronger than the drained-out one, of course
  readsensor();
}

void readsensor()
{
  //if declared outside, the sensor readings become “11111”
  mata0 = analogRead(sensor0);
  mata1 = analogRead(sensor1); 
  mata2 = analogRead(sensor2); 
  mata3 = analogRead(sensor3); 
  mata4 = analogRead(sensor4); 

//converting to digital
if (mata0 < threshold)
  {s[0] = 1;}
else
  {s[0] = 0;}

if (mata1 < threshold)
  {s[1] = 1;}
else
  {s[1] = 0;}

if (mata2 < threshold)
  {s[2] = 1;}
else
  {s[2] = 0;}

if (mata3 < threshold)
  {s[3] = 1;}
else
  {s[3] = 0;}

if (mata4 < threshold)
  {s[4] = 1;}
else
  {s[4] = 0;}

//change the sensor readings into a series of binary number
data=(s[0]*16)+(s[1]*8)+(s[2]*4)+(s[3]*2)+(s[4]*1);

//to display the sensor readings on serial monitor
Serial.print(s[0]);
Serial.print("   ");
Serial.print(s[1]);
Serial.print("   ");
Serial.print(s[2]);
Serial.print("   ");
Serial.print(s[3]);
Serial.print("   ");
Serial.print(s[4]);
Serial.println(" ");
}

//Intersection condition -------------- 0b00abcde
void condition()
{
 if (data==0b0000100)
   {
   moveforward();
   }
 else if (data==0b0000001)
   {
    strongright();
    readsensor();
    while (s[3]==0)
    {
      strongright();
      readsensor();
    }
   }  
 else if (data==0b0000011)
   {midright2();}  
 else if (data==0b0000010)
   {midright();}
 else if (data==0b0000110)
   {softright();}
 else if (data==0b0001100)
   {softleft();}
 else if (data==0b0001000)
   {midleft();}
 else if (data==0b0011000)
   {midleft2();}
 else if (data==0b0010000)
   {
    strongleft();
    readsensor();
    while (s[1]==0)
    {
     strongleft();
     readsensor();
    }
   }
 else //there is a right angle turn or intersection
   {
     
      if (data==0b0011111) //T, +, end of maze
        {
          lilmoveforward();
          if (data==0b0000000)// T intersection
            {
              
              lefttillstraight();
              path[pathlength]='L';pathlength++;//save L
              
            }
            else if (data==0b0000000) //dead end
        {
          turnaround();
          path[pathlength]='U';pathlength++;//save U
          
        }
          else if (data==0b0011111)//end of maze
            {
              stop(); //stopping the robot
              path[pathlength]='F';
              pathlength++;//save F
              
              //sign for the end of maze
              
              
              shortpath(); //calculate the shortest path
              
              
              //sign to prepare the robot (put it back) on the starting position
             
              shortestpath(); //move through the shortest path
            }
        
          else //ada garis ke arah lurus 00100 dll (+ intersection)
            {
              lefttillstraight();
              //turnleft();
              path[pathlength]='L';pathlength++;//save L
              
            }
        }
   
  //straight or right junction
      else if ((data==0b0000111)||(data==0b0001111))
        {
          lilmoveforward();lilmoveforward();
          if (data==0b0000000)//right only
            {
              righttillstraight();
            }
    else if (s[2]==1)
                  {
              
              moveforward();
              path[pathlength]='S';pathlength++;//save S
              
            }
        }
//left or straight junction
      else if ((data==0b0011100)||(data==0b0011110))
        {
        
          
              
              lefttillstraight();
            
        
            
              
              path[pathlength]='L';pathlength++;//save L
              
            }
         }
    }


void shortpath() //calculate the shortest path
{
  //because (..F) is the last and there is no U recorderd before F 
  int x = (pathlength-2);

  while (x > 0)  
  {
    if (path[x]=='U')
      {
        if (path[x-1]=='L' && path[x+1]=='L')
          {path[x-1]='S';path[x]='O';path[x+1]='O';}
        else if (path[x-1]=='L' && path[x+1]=='S')
          {path[x-1]='R';path[x]='O';path[x+1]='O';}
        else if (path[x-1]=='R' && path[x+1]=='R')
          {path[x-1]='S';path[x]='O';path[x+1]='O';}
        else if (path[x-1]=='R' && path[x+1]=='S')
          {path[x-1]='L';path[x]='O';path[x+1]='O';}
        else if (path[x-1]=='S' && path[x+1]=='L')
          {path[x-1]='R';path[x]='O';path[x+1]='O';}
        else if (path[x-1]=='S' && path[x+1]=='R')
          {path[x-1]='L';path[x]='O';path[x+1]='O';}
        else if (path[x-1]=='L' && path[x+1]=='R')
          {
            path[x-1]='U';path[x]='O';path[x+1]='O';
            
          }
        else if (path[x-1]=='R' && path[x+1]=='L')
          {
            path[x-1]='U';path[x]='O';path[x+1]='O';
            
          }
        else if (path[x-1]=='S' && path[x+1]=='S')
          {
            path[x-1]='U';path[x]='O';path[x+1]='O';
            
          }
        //---------------------------------------
        x--;
      }
    else
      {x--;}
  }
}

void shortestpath()
{
 readsensor();
 if (data==0b0000100)
   {moveforward();}
 else if (data==0b0000001)
   {
    strongright();
    readsensor();
    while (s[3]==0)
    {
      strongright();
      readsensor();
    }
   }
 else if (data==0b0000011)
   {midright2();}  
 else if (data==0b0000010)
   {midright();}
 else if (data==0b0000110)
   {softright();}
 else if (data==0b0001100)
   {softleft();}
 else if (data==0b0001000)
   {midleft();}
 else if (data==0b0011000)
   {midleft2();}
 else if (data==0b0010000)
   {
    strongleft();
    readsensor();
    while (s[1]==0)
    {
     strongleft();
     readsensor();
    }
   }
 else
     //right or straight
     if ((data==0b0000111)||(data==0b0001111))
      {
        lilmoveforward();
        if (data==0b0000000)//right only
          {
            righttillstraight();
          }
        else //there is a straight path
          {
            choosepath();
          }
      }
    //left or straight
    else if ((data==0b0011100)||(data==0b0011110))
      {
        
            choosepath();
          
      }
    //T, +, or END OF MAZE
    else if  (data==0b0011111)
      {
        choosepath();
      }
  shortestpath();
}  

void choosepath()//to get rid of the effect of “path[]==0” in the record
{
  if (path[readpath]=='F')
    { 
      stop();
      //finish();
    }
  else if (path[readpath]=='R')
    {
     
      righttillstraight();
    }
  else if (path[readpath]=='S')
    {
      
      moveforward();delay(200);
    }
  else if (path[readpath]=='L')
    {
     
      lefttillstraight();
    }
  else if (path[readpath]=='O')
    {
      readpath++;
      choosepath();
    } 
  readpath++;
  shortestpath();
}

/*void finish()
{
 
  finish();
}
*/
void setup()
{
  Serial.begin(9600);
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);
  pinMode(onR, OUTPUT);
  pinMode(onL, OUTPUT);

  analogReference(INTERNAL);
}

void loop()
{
 readsensor();
 condition();
}
