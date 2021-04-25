// Generated by gencpp from file hebi_cpp_api_examples/StartPath.msg
// DO NOT EDIT!


#ifndef HEBI_CPP_API_EXAMPLES_MESSAGE_STARTPATH_H
#define HEBI_CPP_API_EXAMPLES_MESSAGE_STARTPATH_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hebi_cpp_api_examples
{
template <class ContainerAllocator>
struct StartPath_
{
  typedef StartPath_<ContainerAllocator> Type;

  StartPath_()
    {
    }
  StartPath_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> const> ConstPtr;

}; // struct StartPath_

typedef ::hebi_cpp_api_examples::StartPath_<std::allocator<void> > StartPath;

typedef boost::shared_ptr< ::hebi_cpp_api_examples::StartPath > StartPathPtr;
typedef boost::shared_ptr< ::hebi_cpp_api_examples::StartPath const> StartPathConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace hebi_cpp_api_examples

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::hebi_cpp_api_examples::StartPath_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hebi_cpp_api_examples/StartPath";
  }

  static const char* value(const ::hebi_cpp_api_examples::StartPath_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::hebi_cpp_api_examples::StartPath_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StartPath_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hebi_cpp_api_examples::StartPath_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::hebi_cpp_api_examples::StartPath_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // HEBI_CPP_API_EXAMPLES_MESSAGE_STARTPATH_H