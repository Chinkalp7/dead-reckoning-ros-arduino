/* 
 * rosserial Planar Odometry Example
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

double x = 0.0;                         // Initial X position
double y = 0.0;                         // Initial Y position
double theta = 0.00;                    // Initial Theta angle

char base_link[] = "/base_link";
char odom[] = "/odom";

volatile signed int counterL = 0;       // This variable will increase or decrease depending on the rotation of encoder
volatile signed int counterR = 0;       // This variable will increase or decrease depending on the rotation of encoder
volatile int dcountL = 0;               // diff in encoder reading for left wheel
volatile int dcountR = 0;               // diff in encoder reading for left wheel
float pi=3.14;                          // Pi=3.14
float R=0.115;                          // Wheel Radius 
unsigned int tick=400;                  // Encoder total tick
float len=0.60;                         // Distance between two wheels


void setup()
{
  nh.initNode();                        // Initializing node handler
  broadcaster.init(nh);                 // odom data broadcaster init
  
  pinMode(2, INPUT);
  pinMode(4, INPUT);
  digitalWrite(2, HIGH);                // Left encoder input
  digitalWrite(4, HIGH);
  attachInterrupt(0, ai0, RISING); 

  pinMode(3, INPUT);
  pinMode(5, INPUT);                    // Right encoder input
  digitalWrite(3, HIGH); 
  digitalWrite(5, HIGH); 
  attachInterrupt(1, ai1, RISING);
  
}

void loop()
{ 
  dcountL=counterL-dcountL;
  dcountR=counterR-dcountR;
   
  signed int templ=counterL; 
  signed int tempR=counterR;
  
  float dL=0.72*(dcountL/(float)tick);  // Dl = 2*PI*R*(lefttick/totaltick)
  float dR=0.72*(dcountR/(float)tick);  // Dr = 2*PI*R*(righttick/totaltick)
  float dC=(dL+dR)/2;

   x = x+(dC*(cos(theta)));             // calculates new X position based on wheel revolution
   y = y+(dC*(sin(theta)));             // calculates new Y position based on wheel revolution
   theta = theta +((dR-dL)/len);        // calculates new theta angle based on encoder values
  if(theta > 3.14)
    theta=-3.14;
    
  // tf odom->base_link
  t.header.frame_id = odom;             // odom data publishes on Odom topic
  t.child_frame_id = base_link;         
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  // converting from euler angle to quaternion form
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);         // broadcasting updated result 
  nh.spinOnce();
  
  dcountL=templ;
  dcountR=tempR;
  
  
  delay(50);
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 4 to determine the direction
  if(digitalRead(4)==LOW) {
    counterL++;
  }else{
    counterL--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 5 to determine the direction
  if(digitalRead(5)==LOW) {
    counterR--;
  }else{
    counterR++;
  }
}
