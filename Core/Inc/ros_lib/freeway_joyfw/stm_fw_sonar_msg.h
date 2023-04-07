#ifndef _ROS_freeway_joyfw_stm_fw_sonar_msg_h
#define _ROS_freeway_joyfw_stm_fw_sonar_msg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/Range.h"

namespace freeway_joyfw
{

  class stm_fw_sonar_msg : public ros::Msg
  {
    public:
      typedef sensor_msgs::Range _range_right_type;
      _range_right_type range_right;
      typedef sensor_msgs::Range _range_left_type;
      _range_left_type range_left;

    stm_fw_sonar_msg():
      range_right(),
      range_left()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->range_right.serialize(outbuffer + offset);
      offset += this->range_left.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->range_right.deserialize(inbuffer + offset);
      offset += this->range_left.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "freeway_joyfw/stm_fw_sonar_msg"; };
    virtual const char * getMD5() override { return "91d798d2bd3e20280e142f349dff90b6"; };

  };

}
#endif
