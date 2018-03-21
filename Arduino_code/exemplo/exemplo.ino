#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>

#include <Servo.h>

ros::NodeHandle nh;

std_msgs::Int16MultiArray thermo;
ros::Publisher thermo_pub("thermo", &thermo);

char dim0_label[] = "thermo";
void setup()
{
  nh.initNode();
  thermo.layout.dim = (std_msgs::MultiArrayDimension *)
  malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
  thermo.layout.dim[0].label = dim0_label;
  thermo.layout.dim[0].size = 8;
  thermo.layout.dim[0].stride = 1*8;
  thermo.layout.data_offset = 0;
  
  thermo.layout.dim_length = 1;
  thermo.data_length = 8;
  
  thermo.data = (int *)malloc(sizeof(int)*8);
  nh.advertise(thermo_pub);
}

void loop()
{
  for(int i = 0; i < 8; i++){
    thermo.data[i] = analogRead(i);
  }
  thermo_pub.publish( &thermo );   
  nh.spinOnce();
  delay(500);
}
