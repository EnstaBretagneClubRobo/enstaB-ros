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
 * Auto-generated by genmsg_cpp from file /home/elessog/catkin_ws/src/matrice_from_imu/srv/matFromImu.srv
 *
 */


#ifndef MATRICE_FROM_IMU_MESSAGE_MATFROMIMURESPONSE_H
#define MATRICE_FROM_IMU_MESSAGE_MATFROMIMURESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Transform.h>

namespace matrice_from_imu
{
template <class ContainerAllocator>
struct matFromImuResponse_
{
  typedef matFromImuResponse_<ContainerAllocator> Type;

  matFromImuResponse_()
    : transform()  {
    }
  matFromImuResponse_(const ContainerAllocator& _alloc)
    : transform(_alloc)  {
    }



   typedef  ::geometry_msgs::Transform_<ContainerAllocator>  _transform_type;
  _transform_type transform;




  typedef boost::shared_ptr< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct matFromImuResponse_

typedef ::matrice_from_imu::matFromImuResponse_<std::allocator<void> > matFromImuResponse;

typedef boost::shared_ptr< ::matrice_from_imu::matFromImuResponse > matFromImuResponsePtr;
typedef boost::shared_ptr< ::matrice_from_imu::matFromImuResponse const> matFromImuResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace matrice_from_imu

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/hydro/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c2d1de03cf5b052350d875b7cfbc84a5";
  }

  static const char* value(const ::matrice_from_imu::matFromImuResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc2d1de03cf5b0523ULL;
  static const uint64_t static_value2 = 0x50d875b7cfbc84a5ULL;
};

template<class ContainerAllocator>
struct DataType< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "matrice_from_imu/matFromImuResponse";
  }

  static const char* value(const ::matrice_from_imu::matFromImuResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Transform transform\n\
\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Transform\n\
# This represents the transform between two coordinate frames in free space.\n\
\n\
Vector3 translation\n\
Quaternion rotation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::matrice_from_imu::matFromImuResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.transform);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct matFromImuResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::matrice_from_imu::matFromImuResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::matrice_from_imu::matFromImuResponse_<ContainerAllocator>& v)
  {
    s << indent << "transform: ";
    s << std::endl;
    Printer< ::geometry_msgs::Transform_<ContainerAllocator> >::stream(s, indent + "  ", v.transform);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MATRICE_FROM_IMU_MESSAGE_MATFROMIMURESPONSE_H
