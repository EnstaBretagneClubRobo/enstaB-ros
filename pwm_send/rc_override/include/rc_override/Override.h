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
 * Auto-generated by gensrv_cpp from file /home/nuc1/catkin_ws/src/rc_override/srv/Override.srv
 *
 */


#ifndef RC_OVERRIDE_MESSAGE_OVERRIDE_H
#define RC_OVERRIDE_MESSAGE_OVERRIDE_H

#include <ros/service_traits.h>


#include <rc_override/OverrideRequest.h>
#include <rc_override/OverrideResponse.h>


namespace rc_override
{

struct Override
{

typedef OverrideRequest Request;
typedef OverrideResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Override
} // namespace rc_override


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::rc_override::Override > {
  static const char* value()
  {
    return "477a136a8de4555187228889b2a82d9a";
  }

  static const char* value(const ::rc_override::Override&) { return value(); }
};

template<>
struct DataType< ::rc_override::Override > {
  static const char* value()
  {
    return "rc_override/Override";
  }

  static const char* value(const ::rc_override::Override&) { return value(); }
};


// service_traits::MD5Sum< ::rc_override::OverrideRequest> should match 
// service_traits::MD5Sum< ::rc_override::Override > 
template<>
struct MD5Sum< ::rc_override::OverrideRequest>
{
  static const char* value()
  {
    return MD5Sum< ::rc_override::Override >::value();
  }
  static const char* value(const ::rc_override::OverrideRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::rc_override::OverrideRequest> should match 
// service_traits::DataType< ::rc_override::Override > 
template<>
struct DataType< ::rc_override::OverrideRequest>
{
  static const char* value()
  {
    return DataType< ::rc_override::Override >::value();
  }
  static const char* value(const ::rc_override::OverrideRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::rc_override::OverrideResponse> should match 
// service_traits::MD5Sum< ::rc_override::Override > 
template<>
struct MD5Sum< ::rc_override::OverrideResponse>
{
  static const char* value()
  {
    return MD5Sum< ::rc_override::Override >::value();
  }
  static const char* value(const ::rc_override::OverrideResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::rc_override::OverrideResponse> should match 
// service_traits::DataType< ::rc_override::Override > 
template<>
struct DataType< ::rc_override::OverrideResponse>
{
  static const char* value()
  {
    return DataType< ::rc_override::Override >::value();
  }
  static const char* value(const ::rc_override::OverrideResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // RC_OVERRIDE_MESSAGE_OVERRIDE_H
