// Generated by gencpp from file action_demo/ActionDemoResult.msg
// DO NOT EDIT!


#ifndef ACTION_DEMO_MESSAGE_ACTIONDEMORESULT_H
#define ACTION_DEMO_MESSAGE_ACTIONDEMORESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace action_demo
{
template <class ContainerAllocator>
struct ActionDemoResult_
{
  typedef ActionDemoResult_<ContainerAllocator> Type;

  ActionDemoResult_()
    : result(0)  {
    }
  ActionDemoResult_(const ContainerAllocator& _alloc)
    : result(0)  {
  (void)_alloc;
    }



   typedef int32_t _result_type;
  _result_type result;





  typedef boost::shared_ptr< ::action_demo::ActionDemoResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::action_demo::ActionDemoResult_<ContainerAllocator> const> ConstPtr;

}; // struct ActionDemoResult_

typedef ::action_demo::ActionDemoResult_<std::allocator<void> > ActionDemoResult;

typedef boost::shared_ptr< ::action_demo::ActionDemoResult > ActionDemoResultPtr;
typedef boost::shared_ptr< ::action_demo::ActionDemoResult const> ActionDemoResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::action_demo::ActionDemoResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::action_demo::ActionDemoResult_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::action_demo::ActionDemoResult_<ContainerAllocator1> & lhs, const ::action_demo::ActionDemoResult_<ContainerAllocator2> & rhs)
{
  return lhs.result == rhs.result;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::action_demo::ActionDemoResult_<ContainerAllocator1> & lhs, const ::action_demo::ActionDemoResult_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace action_demo

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::action_demo::ActionDemoResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::action_demo::ActionDemoResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::action_demo::ActionDemoResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::action_demo::ActionDemoResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::action_demo::ActionDemoResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::action_demo::ActionDemoResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::action_demo::ActionDemoResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "034a8e20d6a306665e3a5b340fab3f09";
  }

  static const char* value(const ::action_demo::ActionDemoResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x034a8e20d6a30666ULL;
  static const uint64_t static_value2 = 0x5e3a5b340fab3f09ULL;
};

template<class ContainerAllocator>
struct DataType< ::action_demo::ActionDemoResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "action_demo/ActionDemoResult";
  }

  static const char* value(const ::action_demo::ActionDemoResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::action_demo::ActionDemoResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#最终结果\n"
"int32 result\n"
;
  }

  static const char* value(const ::action_demo::ActionDemoResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::action_demo::ActionDemoResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ActionDemoResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::action_demo::ActionDemoResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::action_demo::ActionDemoResult_<ContainerAllocator>& v)
  {
    s << indent << "result: ";
    Printer<int32_t>::stream(s, indent + "  ", v.result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ACTION_DEMO_MESSAGE_ACTIONDEMORESULT_H
