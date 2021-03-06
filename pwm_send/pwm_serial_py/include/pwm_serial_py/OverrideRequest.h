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
 * Auto-generated by genmsg_cpp from file /home/nuc1/catkin_ws/src/pwm_serial_py/srv/Override.srv
 *
 */


#ifndef PWM_SERIAL_PY_MESSAGE_OVERRIDEREQUEST_H
#define PWM_SERIAL_PY_MESSAGE_OVERRIDEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <mavros/OverrideRCIn.h>

namespace pwm_serial_py
{
template <class ContainerAllocator>
struct OverrideRequest_
{
  typedef OverrideRequest_<ContainerAllocator> Type;

  OverrideRequest_()
    : over_msg()  {
    }
  OverrideRequest_(const ContainerAllocator& _alloc)
    : over_msg(_alloc)  {
    }



   typedef  ::mavros::OverrideRCIn_<ContainerAllocator>  _over_msg_type;
  _over_msg_type over_msg;




  typedef boost::shared_ptr< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct OverrideRequest_

typedef ::pwm_serial_py::OverrideRequest_<std::allocator<void> > OverrideRequest;

typedef boost::shared_ptr< ::pwm_serial_py::OverrideRequest > OverrideRequestPtr;
typedef boost::shared_ptr< ::pwm_serial_py::OverrideRequest const> OverrideRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pwm_serial_py::OverrideRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pwm_serial_py

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/hydro/share/sensor_msgs/cmake/../msg'], 'mavros': ['/opt/ros/hydro/share/mavros/cmake/../msg'], 'geometry_msgs': ['/opt/ros/hydro/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg'], 'diagnostic_msgs': ['/opt/ros/hydro/share/diagnostic_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e1698552eed2b39e5dbaab60e1d608b1";
  }

  static const char* value(const ::pwm_serial_py::OverrideRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe1698552eed2b39eULL;
  static const uint64_t static_value2 = 0x5dbaab60e1d608b1ULL;
};

template<class ContainerAllocator>
struct DataType< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pwm_serial_py/OverrideRequest";
  }

  static const char* value(const ::pwm_serial_py::OverrideRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mavros/OverrideRCIn over_msg\n\
\n\
================================================================================\n\
MSG: mavros/OverrideRCIn\n\
# Override RC Input\n\
# Currently MAVLink defines override for 8 channel\n\
\n\
uint16 CHAN_RELEASE=0\n\
uint16 CHAN_NOCHANGE=65535\n\
\n\
uint16[8] channels\n\
";
  }

  static const char* value(const ::pwm_serial_py::OverrideRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.over_msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct OverrideRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pwm_serial_py::OverrideRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pwm_serial_py::OverrideRequest_<ContainerAllocator>& v)
  {
    s << indent << "over_msg: ";
    s << std::endl;
    Printer< ::mavros::OverrideRCIn_<ContainerAllocator> >::stream(s, indent + "  ", v.over_msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PWM_SERIAL_PY_MESSAGE_OVERRIDEREQUEST_H
