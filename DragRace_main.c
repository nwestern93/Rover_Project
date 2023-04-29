/*

TASK TO COMPLETE:

Drag Race – Our rover will attempt to drive as fast as possible to a finish line and stop as
close to the finish line as possible, again, without us being able to watch the rover while
it drives.

FILE DESCRIPTION:
This file is the main file used to impelment our Drag Race code. 
(Description of drag race task seen above)


*/


/////////////////////////////////START PREPROCESSOR CODE/////////////////////////////
#include "main.h"



/////////////////////////////////END PREPROCESSOR CODE//////////////////////////////


////////////////////////////START FUNCTION DECLARATIONS//////////////////////////////
void SystemClock_Config(void);

////////////////////////////////END FUNCTION DECLARATIONS////////////////////////////



int main(void)
{


  SystemClock_Config();
  
  /////////////////////////START INITIALIZATIONS//////////////////////////////////


 ////////////////////////////END INITIALIZATIONS/////////////////////////////////
  while (1)
  {
   
    //set left right motor to 100% forward
    
     //set right motor to 100% forward
    
    //if (direction of rover changes too fast to the left)     Use I2C GYRO or NEO 6M GPS sensor?
    {
      //slow down right wheel until direction same as original
    }
    
    //else if (direction of rover changes too fast to the right)
    {
      //slow down left wheel until direction same as original
    }
    
    //else if (too much slipping of one wheel occurs)
    {
      //slow down that wheel to attempt to correct straight course
    }
    
    
    
    
  }

}
