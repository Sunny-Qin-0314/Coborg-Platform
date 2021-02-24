#ifndef _ROS_rosserial_arduino_CMU_h
#define _ROS_rosserial_arduino_CMU_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosserial_arduino
{

  class CMU : public ros::Msg
  {
    public:
      typedef uint16_t _Potentiometer_type;
      _Potentiometer_type Potentiometer;
      typedef uint16_t _Flex_Sensor_type;
      _Flex_Sensor_type Flex_Sensor;
      typedef uint16_t _IR_Sensor_type;
      _IR_Sensor_type IR_Sensor;
      typedef uint16_t _Ultrasonic_Sensor_type;
      _Ultrasonic_Sensor_type Ultrasonic_Sensor;
      typedef uint16_t _Button_State_type;
      _Button_State_type Button_State;

    CMU():
      Potentiometer(0),
      Flex_Sensor(0),
      IR_Sensor(0),
      Ultrasonic_Sensor(0),
      Button_State(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->Potentiometer >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Potentiometer >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Potentiometer);
      *(outbuffer + offset + 0) = (this->Flex_Sensor >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Flex_Sensor >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Flex_Sensor);
      *(outbuffer + offset + 0) = (this->IR_Sensor >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->IR_Sensor >> (8 * 1)) & 0xFF;
      offset += sizeof(this->IR_Sensor);
      *(outbuffer + offset + 0) = (this->Ultrasonic_Sensor >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Ultrasonic_Sensor >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Ultrasonic_Sensor);
      *(outbuffer + offset + 0) = (this->Button_State >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Button_State >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Button_State);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->Potentiometer =  ((uint16_t) (*(inbuffer + offset)));
      this->Potentiometer |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Potentiometer);
      this->Flex_Sensor =  ((uint16_t) (*(inbuffer + offset)));
      this->Flex_Sensor |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Flex_Sensor);
      this->IR_Sensor =  ((uint16_t) (*(inbuffer + offset)));
      this->IR_Sensor |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->IR_Sensor);
      this->Ultrasonic_Sensor =  ((uint16_t) (*(inbuffer + offset)));
      this->Ultrasonic_Sensor |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Ultrasonic_Sensor);
      this->Button_State =  ((uint16_t) (*(inbuffer + offset)));
      this->Button_State |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Button_State);
     return offset;
    }

    virtual const char * getType() override { return "rosserial_arduino/CMU"; };
    virtual const char * getMD5() override { return "be9cdfeec1c60f4d26327be218060167"; };

  };

}
#endif
