/**********************************************************************************************************
「Pixy+Dobto+Arduino」デモプログラム
***********************************************************************************************************/
 
 const int MODE = 3;//  0:動作確認用,  1:pixy動作確認用,  2:座標確認用,  3:demo


#include <SPI.h>
#include <Pixy2_rm_zumo.h>

Pixy2 pixy;
#include "stdio.h"
#include "Protocol.h"
#include "command.h"
#include "FlexiTimer2.h"






//Set Serial TX&RX Buffer Size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

//#define JOG_STICK
/*********************************************************************************************************
** Global parameters
*********************************************************************************************************/
EndEffectorParams gEndEffectorParams;

JOGJointParams  gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd          gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd          gPTPCmd;

uint64_t gQueuedCmdIndex;

/*********************************************************************************************************
** Function name:       setup
** Descriptions:        Initializes Serial
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  printf_begin();
  //Set Timer Interrupt
  FlexiTimer2::set(100, Serialread);
  FlexiTimer2::start();

  //ledピン設定
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pixy.init();

  gPTPCmd.x = 200;
  gPTPCmd.y = 0;
  gPTPCmd.z = 0;
  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
  SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
  ProtocolProcess(); Serial.println("/////   0    //////");
  delay(3000);
}

/*********************************************************************************************************
** Function name:       Serialread
** Descriptions:        import data to rxbuffer
** Input parametersnone:
** Output parameters:
** Returned value:
*********************************************************************************************************/
void Serialread()
{
  while (Serial1.available()) {
    uint8_t data = Serial1.read();
    if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
      RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
    }
  }
}
/*********************************************************************************************************
** Function name:       Serial_putc
** Descriptions:        Remap Serial to Printf
** Input parametersnone:
** Output parameters:
** Returned value:
*********************************************************************************************************/
int Serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}

/*********************************************************************************************************
** Function name:       printf_begin
** Descriptions:        Initializes Printf
** Input parameters:
** Output parameters:
** Returned value:
*********************************************************************************************************/
void printf_begin(void)
{
  fdevopen( &Serial_putc, 0 );
}

/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void InitRAM(void)
{
  //Set JOG Model
  gJOGJointParams.velocity[0] = 100;
  gJOGJointParams.velocity[1] = 100;
  gJOGJointParams.velocity[2] = 100;
  gJOGJointParams.velocity[3] = 100;
  gJOGJointParams.acceleration[0] = 80;
  gJOGJointParams.acceleration[1] = 80;
  gJOGJointParams.acceleration[2] = 80;
  gJOGJointParams.acceleration[3] = 80;

  gJOGCoordinateParams.velocity[0] = 100;
  gJOGCoordinateParams.velocity[1] = 100;
  gJOGCoordinateParams.velocity[2] = 100;
  gJOGCoordinateParams.velocity[3] = 100;
  gJOGCoordinateParams.acceleration[0] = 80;
  gJOGCoordinateParams.acceleration[1] = 80;
  gJOGCoordinateParams.acceleration[2] = 80;
  gJOGCoordinateParams.acceleration[3] = 80;

  gJOGCommonParams.velocityRatio = 50;
  gJOGCommonParams.accelerationRatio = 50;

  gJOGCmd.cmd = AP_DOWN;
  gJOGCmd.isJoint = JOINT_MODEL;



  //Set PTP Model
  gPTPCoordinateParams.xyzVelocity = 100;
  gPTPCoordinateParams.rVelocity = 100;
  gPTPCoordinateParams.xyzAcceleration = 80;
  gPTPCoordinateParams.rAcceleration = 80;

  gPTPCommonParams.velocityRatio = 50;
  gPTPCommonParams.accelerationRatio = 50;

  gPTPCmd.ptpMode = MOVL_XYZ;
  gPTPCmd.x = 200;
  gPTPCmd.y = 0;
  gPTPCmd.z = 50;
  gPTPCmd.r = 0;

  gQueuedCmdIndex = 0;


}

// Take the biggest block (blocks[0]) that's been around for at least 30 frames (1/2 second)
// and return its index, otherwise return -1
int16_t acquireBlock()
{
  if (pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age > 30)
    return pixy.ccc.blocks[0].m_index;

  return -1;
}


// Find the block with the given index.  In other words, find the same object in the current
// frame -- not the biggest object, but he object we've locked onto in acquireBlock()
// If it's not in the current frame, return NULL
Block *trackBlock(uint8_t index)
{
  uint8_t i;

  for (i = 0; i < pixy.ccc.numBlocks; i++)
  {
    if (index == pixy.ccc.blocks[i].m_index)
      return &pixy.ccc.blocks[i];
  }

  return NULL;
}


/*********************************************************************************************************
** Function name:       led_on
** Descriptions:        ３色ledを光らす
** Input parameters:    int
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void led_on(int x) {

  if (x == 1) {
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
  }
  else if (x == 2) {
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
  }
  else if (x == 3) {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
  }
  else {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
  }

}





/*********************************************************************************************************
** Function name:       PTPMove
** Descriptions:        指定した座標に移動
** Input parameters:    x座標,y座標,z座標,on/off
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void PTPMove(int x, int y, int z, bool on) {

  gPTPCmd.x = x;
  gPTPCmd.y = y;
  gPTPCmd.z = z;
  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

  if (on & 0x01)SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
  else SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);

  ProtocolProcess();
}

/*********************************************************************************************************
** Function name:       PTPMoveDelta
** Descriptions:        指定分だけ移動
** Input parameters:    Δx,Δy,Δz,on/off
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void PTPMoveDelta(int x, int y, int z, bool on) {

  gPTPCmd.x += x;
  gPTPCmd.y += y;
  gPTPCmd.z += z;
  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

  if (on & 0x01)SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
  else SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);

  ProtocolProcess();
}



/*********************************************************************************************************
** Function name:       Pixy_ytoDobot_x
** Descriptions:        物体までの距離をpixyのy値からDobotの必要なxの移動量に変換
** Input parameters:    pixy_y
** Output parameters:   none
** Returned value:      dobot_x
*********************************************************************************************************/
double Pixy_yToDobot_x(double pixy_y) {

  double dobot_x;
 // Serial.print("********Pixy_y******:::");
 // Serial.println(pixy_y);
  dobot_x = (-pixy_y)*50/82 + 289.6;
//  Serial.print("*******dobot_x******:::");
//  Serial.println(dobot_x);
  return dobot_x;

}


/*********************************************************************************************************
** Function name:       Pixy_xtoDobot_y
** Descriptions:        物体までの距離をpixyのx値からDobotの必要なyの移動量に変換
** Input parameters:    pixy_x
** Output parameters:   none
** Returned value:      dobot_y
*********************************************************************************************************/
double Pixy_xToDobot_y(double pixy_x) {

  double dobot_y;

  return dobot_y = (-pixy_x) *30/50 + 92.4;

}

/*********************************************************************************************************
** Function name:       TriangleError_x,TriangleError_y
** Descriptions:        三角測量により,ボックスの高さによって生じる誤差を計算
** Input parameters:    num
** Output parameters:   error_value
** Returned value:      none
*********************************************************************************************************/

double TriangleError_x(double dobot_x){
  double delta_x;
  delta_x = dobot_x - 210;
  return delta_x*5/27;
}

double TriangleError_y(double dobot_y){
  double delta_y;
  delta_y = dobot_y;
  return delta_y*5/27;
}
  
/*********************************************************************************************************
** Function name:       SuctionLoop  
** Descriptions:        サクションカップonをnum回繰り返す
** Input parameters:    num
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void SuctionLoop(int num){
  
  for(int i=0;i<num;i++){PTPMoveDelta(0, 0, 0, 1);delay(10);}

}


/*********************************************************************************************************
** Function name:       loop
** Descriptions:        Program entry
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

void loop()
{

  int x, y, z;

  InitRAM();

  ProtocolInit();

  SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);

  SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);

  SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);

  printf("\r\n======Enter demo application======\r\n");

  SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

  static uint32_t timer = millis();
  static uint32_t count = 0;

  for (; ;)
  {


    for (; ;)
    {
      
      
      
      
      
      

      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      //動作確認モード
      if (MODE == 0) {
        int MODE0_0 = 0;
        z = 32;
        PTPMove(130, 0, 10, 0);
        while (1) {
          PTPMove(250, -30, 10, 0);
          delay(3000);
          PTPMove(250, -30, -32, 0);
          delay(3000);
  //        PTPMove(292, 0, , 0);
   //       delay(3000);
   /*
         Serial.print("TriangleError_x:");
         Serial.print(TriangleError_x(250));
         Serial.print("  TriangleError_y");
         Serial.println(TriangleError_y(60));
*/
        }
      }
      //pixy確認モード
      if (MODE == 1) {
        PTPMove(130, 0, 10, 0);
        static int16_t index = -1;
        Block *block = NULL;
        delay(1000);
        PTPMove(130, 0, 10, 0);
        
        while (1) {
          pixy.ccc.getBlocks();
          pixy.setLED(0, 255, 0);
          Serial.print("num:");Serial.print(pixy.ccc.numBlocks);
          if (index == -1) // search....
          {
            Serial.println("Searching for block...");
            index = acquireBlock();
            if (index >= 0)Serial.println("Found block!");
          }
          if (index >= 0)block = trackBlock(index);
          if (block)
          {
             
            led_on(block->m_signature );
            Serial.print("  m_width");
            Serial.print(block->m_width);
            Serial.print("  m_height");
            Serial.println(block->m_height);
            
            block->print();
          }
          else // no object detected, stop motors, go into search state
          {
            led_on(0);
            index = -1; // set search state
          }

        }
      }
      if (MODE == 2) //座標確認用
      {
        PTPMove(130, 0, 10, 0);
        
        Serial.println("mode2");
        static int16_t index = -1;
        Block *block = NULL;
        delay(1000);
        PTPMove(130, 0, 10, 0);

        while (1) {
          pixy.ccc.getBlocks();
          //Serial.println(pixy.ccc.numBlocks);
          if (index == -1) // search....
          {
           // Serial.println("Searching for block...");
            index = acquireBlock();
           // if (index >= 0)Serial.println("Found block!");
          }
          if (index >= 0)block = trackBlock(index);
          if (block)
          {
            led_on(block->m_signature );
            //block->print();
          }
          else // no object detected, stop motors, go into search state
          {
            led_on(0);
            index = -1; // set search state
          }
          int v, w;
          v = Pixy_yToDobot_x(block->m_y);
          v -= TriangleError_x(v);
          w = Pixy_xToDobot_y(block->m_x);
          w -= TriangleError_y(w); 
          Serial.print("pixy_y");
          Serial.print(block->m_y);
          Serial.print("  pixy_x");
          Serial.print(block->m_x);
          Serial.print("  dobot_x:");
          Serial.print(v);
          Serial.print("  dobot_y:");
          Serial.println(w);


        }


      }//mode2ここまで

      if (MODE == 3) //demo用
      {
        Serial.println("demo start");
        PTPMove(130, 0, 10, 0);
        static int16_t index = -1;
        Block *block = NULL;
        delay(2000);
        PTPMove(130, 0, 10, 0);
        delay(2000);
        while (1) {
          //PTPMove(150, 0, 50, 0);
          pixy.ccc.getBlocks();
          Serial.println(pixy.ccc.numBlocks);
          if (index == -1) // search....
          {
            Serial.println("Searching for block...");
            index = acquireBlock();
            if (index >= 0)Serial.println("Found block!");
          }
          if (index >= 0)block = trackBlock(index);
          if (block)
          {
            led_on(block->m_signature );
            //block->print();
          }
          else // no object detected, stop motors, go into search state
          {
            led_on(0);
            index = -1; // set search state
          }
          int y_block = 40;
          int z_block = 30;

          static int n[3],

          x = Pixy_yToDobot_x(block->m_y);
          x -= TriangleError_x(x);
          delay(1);
          y = Pixy_xToDobot_y(block->m_x);
          y -= TriangleError_y(y);
          delay(1);
          z = 58;
          Serial.print("block->m_y:");
          Serial.print(block->m_y);
          Serial.print("  block->m_x:");
          Serial.print(block->m_x);
          Serial.print("x:");
          Serial.print(x);
          Serial.print(" y:");
          Serial.println(y);
          x = Pixy_yToDobot_x(block->m_y);

          if (millis() - timer > 1000) {
            if (abs(x) > 300 || abs(y) > 200 || x < 130) {
              Serial.println("error x y");
            }



            else
            {
              //PTPMoveDelta(x, y, 0, 0);
              //delay(100);
              //PTPMoveDelta(0, 0, 0, 0);
              //delay(1000);
              PTPMove(x, y, 20-z, 0);
              delay(1000);
              PTPMoveDelta(0, 0, -10, 0);
              delay(1000);
              
              //for(int i=0;i<10;i++){PTPMoveDelta(0, 0, 0, 1);delay(10);}
              SuctionLoop(10);
              delay(100);
              SuctionLoop(10);

              delay(1000);
              SuctionLoop(5);
              PTPMoveDelta(0, 0, z, 1);
              SuctionLoop(5);
              delay(1000);
              SuctionLoop(5);

              
         
              if (block->m_signature == 1) {
                PTPMove(50, -200, 10, 1);
                delay(10);
                PTPMoveDelta(0, 0, 0, 1);
                if(n[0]<4)n[0]++;
              }
              else if (block->m_signature == 2) {
                PTPMove(90, -200,10, 1);
                delay(10);
                PTPMoveDelta(0, 0, 0 ,1);
                if(n[1]<4)n[1]++;
              }
              else {
                PTPMove(130, -200, 10, 1);
                delay(10);
                PTPMoveDelta(0, 0, 0, 1);
                if(n[2]<4)n[2]++;
              }
              delay(100);
              SuctionLoop(5);
              delay(1000);
              if(block->m_signature == 1){PTPMoveDelta(0, 0, -z, 1);SuctionLoop(5);delay(10);}
              else if(block->m_signature == 2){PTPMoveDelta(0, 0, -z, 1);SuctionLoop(5);delay(10);}
              else {PTPMoveDelta(0, 0,-z, 1);SuctionLoop(5);SuctionLoop(5);delay(10);}
              delay(1000);
              
              PTPMoveDelta(0, 0, 0, 1);
              delay(100);
              for(int i=0;i<20;i++){delay(10);PTPMoveDelta(0, 0, 0, 0);}
         
              delay(2000);
              for(int i=0;i<20;i++){delay(10);PTPMoveDelta(0, 0, 0, 0);}
              delay(1000);
              PTPMoveDelta(0, 0, 10, 0);//一回あげる（そうしないとブロックがずれる)
              delay(500);
              PTPMove(90, -190, 10, 0);
              delay(1000);
              PTPMove(150, 0, 10, 0);//いきなりホームに戻ろうとすると、DOBOTの範囲外にいく
              delay(1000);
              PTPMove(130, 0, 10, 0);//ホームポジション
              delay(2000);
            }

            timer = millis();


          }
        }


      }//mode3ここまで


    }





    // Serial.println("hello");
#ifdef JOG_STICK
    // Serial.println("hello");
    if (millis() - timer > 1000)
    {
      timer = millis();
      count++;
      switch (count) {
        case 1:
          gJOGCmd.cmd = AP_DOWN;
          gJOGCmd.isJoint = JOINT_MODEL;
          SetJOGCmd(&gJOGCmd, true, &gQueuedCmdIndex);
          break;
        case 2:
          gJOGCmd.cmd = IDEL;
          gJOGCmd.isJoint = JOINT_MODEL;
          SetJOGCmd(&gJOGCmd, true, &gQueuedCmdIndex);
          break;
        case 3:
          gJOGCmd.cmd = AN_DOWN;
          gJOGCmd.isJoint = JOINT_MODEL;
          SetJOGCmd(&gJOGCmd, true, &gQueuedCmdIndex);
          break;
        case 4:
          gJOGCmd.cmd = IDEL;
          gJOGCmd.isJoint = JOINT_MODEL;
          SetJOGCmd(&gJOGCmd, true, &gQueuedCmdIndex);
          break;
        default:
          count = 0;
          break;
      }
    }
#else
    int i;
    pixy.ccc.getBlocks();
    Serial.println(pixy.ccc.numBlocks);


    static int16_t index = -1;
    Block *block = NULL;

    pixy.ccc.getBlocks();
    Serial.println(pixy.ccc.numBlocks);

    if (index == -1) // search....
    {
      Serial.println("Searching for block...");
      index = acquireBlock();
      if (index >= 0)Serial.println("Found block!");
    }

    // If we've found a block, find it, track it
    if (index >= 0)block = trackBlock(index);

    if (block)
    {
      // calculate pan and tilt errors
      // panOffset = (int32_t)pixy.frameWidth/2 - (int32_t)block->m_x;
      //  tiltOffset = (int32_t)block->m_y - (int32_t)pixy.frameHeight/2;

      led_on(block->m_signature );

      // print the block we're tracking -- wait until end of loop to reduce latency
      // block->print();
    }
    else // no object detected, stop motors, go into search state
    {

      led_on(0);
      index = -1; // set search state
    }



    /////////ここから下にpixyの情報に対するdobotの動作を書く////////////////
    if (millis() - timer > 10000) //dobotにコマンドを送る周期,速くしすぎはだめdobotに動作が蓄積される
    {

      //Serial.println("hello");
      timer = millis();



      //while (1) {



      //pixyの情報から移動量を計算
      int move_x, move_y, move_z;
      move_x = -(int(block -> m_y) - 108) / 2.2 + 90;//block -> m_y:追従物体のpixy座標のy
      move_y = (158 - int(block -> m_x)) / 2.9;
      move_z = -47 - 50;

      gPTPCmd.x += move_x;
      gPTPCmd.y += move_y;
      gPTPCmd.z += 0;
      SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
      ProtocolProcess(); Serial.println("/////   1    //////");
      /*
              gPTPCmd.x += -(int(block -> m_y) - 108) / 2.2 + 90;
              gPTPCmd.y += (158 - int(block -> m_x)) / 2.9 ;
              gPTPCmd.z += 0;
              SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
              ProtocolProcess();
      */
      delay(1000);
      gPTPCmd.x += 0;
      gPTPCmd.y += 0;
      gPTPCmd.z += move_z;
      SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

      //Serial.println("");
      //Serial.println("SuctionCup ON");
      SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
      ProtocolProcess(); Serial.println("/////   2    //////");

      delay(3000);
      gPTPCmd.x += 0;
      gPTPCmd.y += 0;
      gPTPCmd.z += -move_z;
      SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
      //Serial.println("");
      //Serial.println("SuctionCup ON");
      SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
      ProtocolProcess(); Serial.println("/////   3    //////");


      Serial.println("");
      Serial.print("move_x:");
      Serial.println(move_x);
      Serial.print("move_y:");
      Serial.println(move_y);
      Serial.print("move_z:");
      Serial.println(move_z);
      // go_block_position(pixy.ccc.blocks[1].m_x, pixy.ccc.blocks[1].m_y);
      // go_block_position(int(block -> m_x), 104);

      gPTPCmd.x += -move_x;
      gPTPCmd.y += -move_y;
      gPTPCmd.z += 0;
      SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
      //Serial.println("");
      //Serial.println("SuctionCup On");
      SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
      ProtocolProcess(); Serial.println("/////   4    //////");
      delay(3000);

      gPTPCmd.x += 0;
      gPTPCmd.y += 0;
      gPTPCmd.z += move_z;
      SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
      //Serial.println("");
      //Serial.println("SuctionCup On");
      SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
      ProtocolProcess(); Serial.println("/////   5    //////");
      delay(3000);

      //Serial.println("");
      //Serial.println("SuctionCup ON");
      SetEndEffectorSuctionCup(false, true, &gQueuedCmdIndex);
      ProtocolProcess(); Serial.println("/////   6    //////");
      delay(3000);

      gPTPCmd.x += 0;
      gPTPCmd.y += 0;
      gPTPCmd.z += -move_z;
      SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
      //Serial.println("");
      //Serial.println("SuctionCup On");
      SetEndEffectorSuctionCup(true, true, &gQueuedCmdIndex);
      ProtocolProcess(); Serial.println("/////   7    //////");
      delay(3000);
      //while (1) {}
      //}





      ////pixyの座標を見る///
      /*
      Serial.print("x:");
      Serial.print(block -> m_x);
      Serial.print("y:");
      Serial.println(block -> m_y);
      */



      ///////////dobotが動くかチェックする///////////////
      /*
      count++;      if (count & 0x01)
      {
        gPTPCmd.x += 10;
        // gPTPCmd.y += 10;
        // gPTPCmd.z += 10;
        SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
        Serial.println("hello1!");
        //ProtocolProcess();
        //led_on(1);
        //led_on(2);
        //led_on(3);
      }
      else
      {
        gPTPCmd.x -= 10;
        // gPTPCmd.y -= 10;
        // gPTPCmd.z -= 10;
        SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);
      }
      */
      ///////////////////////////////////////
    }
#endif
    //ProtocolProcess();
  }
}






