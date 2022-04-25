// Generated by gencpp from file spark_backend/VisionClientServerRequest.msg
// DO NOT EDIT!


#ifndef SPARK_BACKEND_MESSAGE_VISIONCLIENTSERVERREQUEST_H
#define SPARK_BACKEND_MESSAGE_VISIONCLIENTSERVERREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace spark_backend
{
template <class ContainerAllocator>
struct VisionClientServerRequest_
{
  typedef VisionClientServerRequest_<ContainerAllocator> Type;

  VisionClientServerRequest_()
    {
    }
  VisionClientServerRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> const> ConstPtr;

}; // struct VisionClientServerRequest_

typedef ::spark_backend::VisionClientServerRequest_<std::allocator<void> > VisionClientServerRequest;

typedef boost::shared_ptr< ::spark_backend::VisionClientServerRequest > VisionClientServerRequestPtr;
typedef boost::shared_ptr< ::spark_backend::VisionClientServerRequest const> VisionClientServerRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::spark_backend::VisionClientServerRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace spark_backend

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::spark_backend::VisionClientServerRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "spark_backend/VisionClientServerRequest";
  }

  static const char* value(const ::spark_backend::VisionClientServerRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# client request\n"
;
  }

  static const char* value(const ::spark_backend::VisionClientServerRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VisionClientServerRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::spark_backend::VisionClientServerRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::spark_backend::VisionClientServerRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // SPARK_BACKEND_MESSAGE_VISIONCLIENTSERVERREQUEST_H
