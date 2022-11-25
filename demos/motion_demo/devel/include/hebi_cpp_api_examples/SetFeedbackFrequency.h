// Generated by gencpp from file hebi_cpp_api_examples/SetFeedbackFrequency.msg
// DO NOT EDIT!


#ifndef HEBI_CPP_API_EXAMPLES_MESSAGE_SETFEEDBACKFREQUENCY_H
#define HEBI_CPP_API_EXAMPLES_MESSAGE_SETFEEDBACKFREQUENCY_H

#include <ros/service_traits.h>


#include <hebi_cpp_api_examples/SetFeedbackFrequencyRequest.h>
#include <hebi_cpp_api_examples/SetFeedbackFrequencyResponse.h>


namespace hebi_cpp_api_examples
{

struct SetFeedbackFrequency
{

typedef SetFeedbackFrequencyRequest Request;
typedef SetFeedbackFrequencyResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetFeedbackFrequency
} // namespace hebi_cpp_api_examples


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::hebi_cpp_api_examples::SetFeedbackFrequency > {
  static const char* value()
  {
    return "b11146fd2143e78325a7496114ee3a9e";
  }

  static const char* value(const ::hebi_cpp_api_examples::SetFeedbackFrequency&) { return value(); }
};

template<>
struct DataType< ::hebi_cpp_api_examples::SetFeedbackFrequency > {
  static const char* value()
  {
    return "hebi_cpp_api_examples/SetFeedbackFrequency";
  }

  static const char* value(const ::hebi_cpp_api_examples::SetFeedbackFrequency&) { return value(); }
};


// service_traits::MD5Sum< ::hebi_cpp_api_examples::SetFeedbackFrequencyRequest> should match
// service_traits::MD5Sum< ::hebi_cpp_api_examples::SetFeedbackFrequency >
template<>
struct MD5Sum< ::hebi_cpp_api_examples::SetFeedbackFrequencyRequest>
{
  static const char* value()
  {
    return MD5Sum< ::hebi_cpp_api_examples::SetFeedbackFrequency >::value();
  }
  static const char* value(const ::hebi_cpp_api_examples::SetFeedbackFrequencyRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::hebi_cpp_api_examples::SetFeedbackFrequencyRequest> should match
// service_traits::DataType< ::hebi_cpp_api_examples::SetFeedbackFrequency >
template<>
struct DataType< ::hebi_cpp_api_examples::SetFeedbackFrequencyRequest>
{
  static const char* value()
  {
    return DataType< ::hebi_cpp_api_examples::SetFeedbackFrequency >::value();
  }
  static const char* value(const ::hebi_cpp_api_examples::SetFeedbackFrequencyRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::hebi_cpp_api_examples::SetFeedbackFrequencyResponse> should match
// service_traits::MD5Sum< ::hebi_cpp_api_examples::SetFeedbackFrequency >
template<>
struct MD5Sum< ::hebi_cpp_api_examples::SetFeedbackFrequencyResponse>
{
  static const char* value()
  {
    return MD5Sum< ::hebi_cpp_api_examples::SetFeedbackFrequency >::value();
  }
  static const char* value(const ::hebi_cpp_api_examples::SetFeedbackFrequencyResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::hebi_cpp_api_examples::SetFeedbackFrequencyResponse> should match
// service_traits::DataType< ::hebi_cpp_api_examples::SetFeedbackFrequency >
template<>
struct DataType< ::hebi_cpp_api_examples::SetFeedbackFrequencyResponse>
{
  static const char* value()
  {
    return DataType< ::hebi_cpp_api_examples::SetFeedbackFrequency >::value();
  }
  static const char* value(const ::hebi_cpp_api_examples::SetFeedbackFrequencyResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // HEBI_CPP_API_EXAMPLES_MESSAGE_SETFEEDBACKFREQUENCY_H