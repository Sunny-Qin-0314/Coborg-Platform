// Generated by gencpp from file hebi_cpp_api_examples/BaseMotionAction.msg
// DO NOT EDIT!


#ifndef HEBI_CPP_API_EXAMPLES_MESSAGE_BASEMOTIONACTION_H
#define HEBI_CPP_API_EXAMPLES_MESSAGE_BASEMOTIONACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <hebi_cpp_api_examples/BaseMotionActionGoal.h>
#include <hebi_cpp_api_examples/BaseMotionActionResult.h>
#include <hebi_cpp_api_examples/BaseMotionActionFeedback.h>

namespace hebi_cpp_api_examples
{
template <class ContainerAllocator>
struct BaseMotionAction_
{
  typedef BaseMotionAction_<ContainerAllocator> Type;

  BaseMotionAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  BaseMotionAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::hebi_cpp_api_examples::BaseMotionActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::hebi_cpp_api_examples::BaseMotionActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::hebi_cpp_api_examples::BaseMotionActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef boost::shared_ptr< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> const> ConstPtr;

}; // struct BaseMotionAction_

typedef ::hebi_cpp_api_examples::BaseMotionAction_<std::allocator<void> > BaseMotionAction;

typedef boost::shared_ptr< ::hebi_cpp_api_examples::BaseMotionAction > BaseMotionActionPtr;
typedef boost::shared_ptr< ::hebi_cpp_api_examples::BaseMotionAction const> BaseMotionActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator1> & lhs, const ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator2> & rhs)
{
  return lhs.action_goal == rhs.action_goal &&
    lhs.action_result == rhs.action_result &&
    lhs.action_feedback == rhs.action_feedback;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator1> & lhs, const ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hebi_cpp_api_examples

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0c6552db8de7f3f20933c9e2c2daccd2";
  }

  static const char* value(const ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0c6552db8de7f3f2ULL;
  static const uint64_t static_value2 = 0x0933c9e2c2daccd2ULL;
};

template<class ContainerAllocator>
struct DataType< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hebi_cpp_api_examples/BaseMotionAction";
  }

  static const char* value(const ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"BaseMotionActionGoal action_goal\n"
"BaseMotionActionResult action_result\n"
"BaseMotionActionFeedback action_feedback\n"
"\n"
"================================================================================\n"
"MSG: hebi_cpp_api_examples/BaseMotionActionGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalID goal_id\n"
"BaseMotionGoal goal\n"
"\n"
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
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalID\n"
"# The stamp should store the time at which this goal was requested.\n"
"# It is used by an action server when it tries to preempt all\n"
"# goals that were requested before a certain time\n"
"time stamp\n"
"\n"
"# The id provides a way to associate feedback and\n"
"# result message with specific goal requests. The id\n"
"# specified must be unique.\n"
"string id\n"
"\n"
"\n"
"================================================================================\n"
"MSG: hebi_cpp_api_examples/BaseMotionGoal\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# Goal position (relative to start)\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
"\n"
"# Optionally, set a color when doing the move; otherwise, clear the color.\n"
"bool set_color\n"
"uint8 r\n"
"uint8 g\n"
"uint8 b\n"
"\n"
"================================================================================\n"
"MSG: hebi_cpp_api_examples/BaseMotionActionResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"BaseMotionResult result\n"
"\n"
"================================================================================\n"
"MSG: actionlib_msgs/GoalStatus\n"
"GoalID goal_id\n"
"uint8 status\n"
"uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n"
"uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n"
"uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n"
"                            #   and has since completed its execution (Terminal State)\n"
"uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n"
"uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n"
"                            #    to some failure (Terminal State)\n"
"uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n"
"                            #    because the goal was unattainable or invalid (Terminal State)\n"
"uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n"
"                            #    and has not yet completed execution\n"
"uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n"
"                            #    but the action server has not yet confirmed that the goal is canceled\n"
"uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n"
"                            #    and was successfully cancelled (Terminal State)\n"
"uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n"
"                            #    sent over the wire by an action server\n"
"\n"
"#Allow for the user to associate a string with GoalStatus for debugging\n"
"string text\n"
"\n"
"\n"
"================================================================================\n"
"MSG: hebi_cpp_api_examples/BaseMotionResult\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"bool success\n"
"\n"
"================================================================================\n"
"MSG: hebi_cpp_api_examples/BaseMotionActionFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"Header header\n"
"actionlib_msgs/GoalStatus status\n"
"BaseMotionFeedback feedback\n"
"\n"
"================================================================================\n"
"MSG: hebi_cpp_api_examples/BaseMotionFeedback\n"
"# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"float64 percent_complete\n"
"\n"
;
  }

  static const char* value(const ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BaseMotionAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hebi_cpp_api_examples::BaseMotionAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::hebi_cpp_api_examples::BaseMotionActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::hebi_cpp_api_examples::BaseMotionActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::hebi_cpp_api_examples::BaseMotionActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HEBI_CPP_API_EXAMPLES_MESSAGE_BASEMOTIONACTION_H
