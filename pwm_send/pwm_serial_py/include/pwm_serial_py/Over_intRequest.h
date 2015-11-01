/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/nuc1/catkin_ws/src/pwm_send/pwm_serial_py/srv/Over_int.srv
 *
 */


#ifndef PWM_SERIAL_PY_MESSAGE_OVER_INTREQUEST_H
#define PWM_SERIAL_PY_MESSAGE_OVER_INTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pwm_serial_py
{
template <class ContainerAllocator>
struct Over_intRequest_
{
  typedef Over_intRequest_<ContainerAllocator> Type;

  Over_intRequest_()
    : over_msg()  {
      over_msg.assign(0);
  }
  Over_intRequest_(const ContainerAllocator& _alloc)
    : over_msg()  {
      over_msg.assign(0);
  }



   typedef boost::array<uint16_t, 8>  _over_msg_type;
  _over_msg_type over_msg;




  typedef boost::shared_ptr< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct Over_intRequest_

typedef ::pwm_serial_py::Over_intRequest_<std::allocator<void> > Over_intRequest;

typedef boost::shared_ptr< ::pwm_serial_py::Over_intRequest > Over_intRequestPtr;
typedef boost::shared_ptr< ::pwm_serial_py::Over_intRequest const> Over_intRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pwm_serial_py::Over_intRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pwm_serial_py

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "06537af067b4f4b345b6ac032d7ce145";
  }

  static const char* value(const ::pwm_serial_py::Over_intRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x06537af067b4f4b3ULL;
  static const uint64_t static_value2 = 0x45b6ac032d7ce145ULL;
};

template<class ContainerAllocator>
struct DataType< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pwm_serial_py/Over_intRequest";
  }

  static const char* value(const ::pwm_serial_py::Over_intRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint16[8] over_msg\n\
";
  }

  static const char* value(const ::pwm_serial_py::Over_intRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.over_msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Over_intRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pwm_serial_py::Over_intRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pwm_serial_py::Over_intRequest_<ContainerAllocator>& v)
  {
    s << indent << "over_msg[]" << std::endl;
    for (size_t i = 0; i < v.over_msg.size(); ++i)
    {
      s << indent << "  over_msg[" << i << "]: ";
      Printer<uint16_t>::stream(s, indent + "  ", v.over_msg[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PWM_SERIAL_PY_MESSAGE_OVER_INTREQUEST_H