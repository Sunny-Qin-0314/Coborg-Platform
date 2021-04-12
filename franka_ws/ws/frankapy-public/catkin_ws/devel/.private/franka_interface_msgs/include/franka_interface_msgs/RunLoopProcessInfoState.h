// Generated by gencpp from file franka_interface_msgs/RunLoopProcessInfoState.msg
// DO NOT EDIT!


#ifndef FRANKA_INTERFACE_MSGS_MESSAGE_RUNLOOPPROCESSINFOSTATE_H
#define FRANKA_INTERFACE_MSGS_MESSAGE_RUNLOOPPROCESSINFOSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace franka_interface_msgs
{
template <class ContainerAllocator>
struct RunLoopProcessInfoState_
{
  typedef RunLoopProcessInfoState_<ContainerAllocator> Type;

  RunLoopProcessInfoState_()
    : header()
    , current_memory_region(0)
    , current_sensor_region(0)
    , current_feedback_region(0)
    , current_skill_id(0)
    , current_skill_type(0)
    , current_meta_skill_id(0)
    , current_meta_skill_type(0)
    , current_skill_description()
    , new_skill_available(false)
    , new_skill_id(0)
    , new_skill_type(0)
    , new_meta_skill_id(0)
    , new_meta_skill_type(0)
    , new_skill_description()
    , is_running_skill(false)
    , skill_preempted(false)
    , done_skill_id(0)
    , result_skill_id(0)
    , time_skill_started_in_robot_time(0.0)
    , time_skill_finished_in_robot_time(0.0)
    , is_fresh(false)  {
    }
  RunLoopProcessInfoState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , current_memory_region(0)
    , current_sensor_region(0)
    , current_feedback_region(0)
    , current_skill_id(0)
    , current_skill_type(0)
    , current_meta_skill_id(0)
    , current_meta_skill_type(0)
    , current_skill_description(_alloc)
    , new_skill_available(false)
    , new_skill_id(0)
    , new_skill_type(0)
    , new_meta_skill_id(0)
    , new_meta_skill_type(0)
    , new_skill_description(_alloc)
    , is_running_skill(false)
    , skill_preempted(false)
    , done_skill_id(0)
    , result_skill_id(0)
    , time_skill_started_in_robot_time(0.0)
    , time_skill_finished_in_robot_time(0.0)
    , is_fresh(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int64_t _current_memory_region_type;
  _current_memory_region_type current_memory_region;

   typedef int64_t _current_sensor_region_type;
  _current_sensor_region_type current_sensor_region;

   typedef int64_t _current_feedback_region_type;
  _current_feedback_region_type current_feedback_region;

   typedef int64_t _current_skill_id_type;
  _current_skill_id_type current_skill_id;

   typedef int64_t _current_skill_type_type;
  _current_skill_type_type current_skill_type;

   typedef int64_t _current_meta_skill_id_type;
  _current_meta_skill_id_type current_meta_skill_id;

   typedef int64_t _current_meta_skill_type_type;
  _current_meta_skill_type_type current_meta_skill_type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _current_skill_description_type;
  _current_skill_description_type current_skill_description;

   typedef uint8_t _new_skill_available_type;
  _new_skill_available_type new_skill_available;

   typedef int64_t _new_skill_id_type;
  _new_skill_id_type new_skill_id;

   typedef int64_t _new_skill_type_type;
  _new_skill_type_type new_skill_type;

   typedef int64_t _new_meta_skill_id_type;
  _new_meta_skill_id_type new_meta_skill_id;

   typedef int64_t _new_meta_skill_type_type;
  _new_meta_skill_type_type new_meta_skill_type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _new_skill_description_type;
  _new_skill_description_type new_skill_description;

   typedef uint8_t _is_running_skill_type;
  _is_running_skill_type is_running_skill;

   typedef uint8_t _skill_preempted_type;
  _skill_preempted_type skill_preempted;

   typedef int64_t _done_skill_id_type;
  _done_skill_id_type done_skill_id;

   typedef int64_t _result_skill_id_type;
  _result_skill_id_type result_skill_id;

   typedef double _time_skill_started_in_robot_time_type;
  _time_skill_started_in_robot_time_type time_skill_started_in_robot_time;

   typedef double _time_skill_finished_in_robot_time_type;
  _time_skill_finished_in_robot_time_type time_skill_finished_in_robot_time;

   typedef uint8_t _is_fresh_type;
  _is_fresh_type is_fresh;





  typedef boost::shared_ptr< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> const> ConstPtr;

}; // struct RunLoopProcessInfoState_

typedef ::franka_interface_msgs::RunLoopProcessInfoState_<std::allocator<void> > RunLoopProcessInfoState;

typedef boost::shared_ptr< ::franka_interface_msgs::RunLoopProcessInfoState > RunLoopProcessInfoStatePtr;
typedef boost::shared_ptr< ::franka_interface_msgs::RunLoopProcessInfoState const> RunLoopProcessInfoStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator1> & lhs, const ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.current_memory_region == rhs.current_memory_region &&
    lhs.current_sensor_region == rhs.current_sensor_region &&
    lhs.current_feedback_region == rhs.current_feedback_region &&
    lhs.current_skill_id == rhs.current_skill_id &&
    lhs.current_skill_type == rhs.current_skill_type &&
    lhs.current_meta_skill_id == rhs.current_meta_skill_id &&
    lhs.current_meta_skill_type == rhs.current_meta_skill_type &&
    lhs.current_skill_description == rhs.current_skill_description &&
    lhs.new_skill_available == rhs.new_skill_available &&
    lhs.new_skill_id == rhs.new_skill_id &&
    lhs.new_skill_type == rhs.new_skill_type &&
    lhs.new_meta_skill_id == rhs.new_meta_skill_id &&
    lhs.new_meta_skill_type == rhs.new_meta_skill_type &&
    lhs.new_skill_description == rhs.new_skill_description &&
    lhs.is_running_skill == rhs.is_running_skill &&
    lhs.skill_preempted == rhs.skill_preempted &&
    lhs.done_skill_id == rhs.done_skill_id &&
    lhs.result_skill_id == rhs.result_skill_id &&
    lhs.time_skill_started_in_robot_time == rhs.time_skill_started_in_robot_time &&
    lhs.time_skill_finished_in_robot_time == rhs.time_skill_finished_in_robot_time &&
    lhs.is_fresh == rhs.is_fresh;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator1> & lhs, const ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace franka_interface_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "86cd879bb4a3cbdb82beb45c42836130";
  }

  static const char* value(const ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x86cd879bb4a3cbdbULL;
  static const uint64_t static_value2 = 0x82beb45c42836130ULL;
};

template<class ContainerAllocator>
struct DataType< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "franka_interface_msgs/RunLoopProcessInfoState";
  }

  static const char* value(const ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Skill state\n"
"std_msgs/Header header\n"
"int64 current_memory_region\n"
"int64 current_sensor_region\n"
"int64 current_feedback_region\n"
"int64 current_skill_id \n"
"int64 current_skill_type\n"
"int64 current_meta_skill_id\n"
"int64 current_meta_skill_type\n"
"string current_skill_description\n"
"bool new_skill_available\n"
"int64 new_skill_id\n"
"int64 new_skill_type\n"
"int64 new_meta_skill_id\n"
"int64 new_meta_skill_type\n"
"string new_skill_description\n"
"bool is_running_skill\n"
"bool skill_preempted\n"
"int64 done_skill_id\n"
"int64 result_skill_id\n"
"float64 time_skill_started_in_robot_time\n"
"float64 time_skill_finished_in_robot_time\n"
"bool is_fresh\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.current_memory_region);
      stream.next(m.current_sensor_region);
      stream.next(m.current_feedback_region);
      stream.next(m.current_skill_id);
      stream.next(m.current_skill_type);
      stream.next(m.current_meta_skill_id);
      stream.next(m.current_meta_skill_type);
      stream.next(m.current_skill_description);
      stream.next(m.new_skill_available);
      stream.next(m.new_skill_id);
      stream.next(m.new_skill_type);
      stream.next(m.new_meta_skill_id);
      stream.next(m.new_meta_skill_type);
      stream.next(m.new_skill_description);
      stream.next(m.is_running_skill);
      stream.next(m.skill_preempted);
      stream.next(m.done_skill_id);
      stream.next(m.result_skill_id);
      stream.next(m.time_skill_started_in_robot_time);
      stream.next(m.time_skill_finished_in_robot_time);
      stream.next(m.is_fresh);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RunLoopProcessInfoState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::franka_interface_msgs::RunLoopProcessInfoState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "current_memory_region: ";
    Printer<int64_t>::stream(s, indent + "  ", v.current_memory_region);
    s << indent << "current_sensor_region: ";
    Printer<int64_t>::stream(s, indent + "  ", v.current_sensor_region);
    s << indent << "current_feedback_region: ";
    Printer<int64_t>::stream(s, indent + "  ", v.current_feedback_region);
    s << indent << "current_skill_id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.current_skill_id);
    s << indent << "current_skill_type: ";
    Printer<int64_t>::stream(s, indent + "  ", v.current_skill_type);
    s << indent << "current_meta_skill_id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.current_meta_skill_id);
    s << indent << "current_meta_skill_type: ";
    Printer<int64_t>::stream(s, indent + "  ", v.current_meta_skill_type);
    s << indent << "current_skill_description: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.current_skill_description);
    s << indent << "new_skill_available: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.new_skill_available);
    s << indent << "new_skill_id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.new_skill_id);
    s << indent << "new_skill_type: ";
    Printer<int64_t>::stream(s, indent + "  ", v.new_skill_type);
    s << indent << "new_meta_skill_id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.new_meta_skill_id);
    s << indent << "new_meta_skill_type: ";
    Printer<int64_t>::stream(s, indent + "  ", v.new_meta_skill_type);
    s << indent << "new_skill_description: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.new_skill_description);
    s << indent << "is_running_skill: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_running_skill);
    s << indent << "skill_preempted: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.skill_preempted);
    s << indent << "done_skill_id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.done_skill_id);
    s << indent << "result_skill_id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.result_skill_id);
    s << indent << "time_skill_started_in_robot_time: ";
    Printer<double>::stream(s, indent + "  ", v.time_skill_started_in_robot_time);
    s << indent << "time_skill_finished_in_robot_time: ";
    Printer<double>::stream(s, indent + "  ", v.time_skill_finished_in_robot_time);
    s << indent << "is_fresh: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_fresh);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FRANKA_INTERFACE_MSGS_MESSAGE_RUNLOOPPROCESSINFOSTATE_H
