/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: TwistStamped_.idl
  Source: TwistStamped_.hpp
  Cyclone DDS: v0.10.2

*****************************************************************/
#ifndef DDSCXX_UNITREE_IDL_ROS2_TWISTSTAMPED__HPP
#define DDSCXX_UNITREE_IDL_ROS2_TWISTSTAMPED__HPP

#include "unitree/idl/ros2/Twist_.hpp"

#include "unitree/idl/ros2/Header_.hpp"


namespace geometry_msgs
{
namespace msg
{
namespace dds_
{
class TwistStamped_
{
private:
 ::std_msgs::msg::dds_::Header_ header_;
 ::geometry_msgs::msg::dds_::Twist_ twist_;

public:
  TwistStamped_() = default;

  explicit TwistStamped_(
    const ::std_msgs::msg::dds_::Header_& header,
    const ::geometry_msgs::msg::dds_::Twist_& twist) :
    header_(header),
    twist_(twist) { }

  const ::std_msgs::msg::dds_::Header_& header() const { return this->header_; }
  ::std_msgs::msg::dds_::Header_& header() { return this->header_; }
  void header(const ::std_msgs::msg::dds_::Header_& _val_) { this->header_ = _val_; }
  void header(::std_msgs::msg::dds_::Header_&& _val_) { this->header_ = _val_; }
  const ::geometry_msgs::msg::dds_::Twist_& twist() const { return this->twist_; }
  ::geometry_msgs::msg::dds_::Twist_& twist() { return this->twist_; }
  void twist(const ::geometry_msgs::msg::dds_::Twist_& _val_) { this->twist_ = _val_; }
  void twist(::geometry_msgs::msg::dds_::Twist_&& _val_) { this->twist_ = _val_; }

  bool operator==(const TwistStamped_& _other) const
  {
    (void) _other;
    return header_ == _other.header_ &&
      twist_ == _other.twist_;
  }

  bool operator!=(const TwistStamped_& _other) const
  {
    return !(*this == _other);
  }

};

}

}

}

#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace topic {

template <> constexpr const char* TopicTraits<::geometry_msgs::msg::dds_::TwistStamped_>::getTypeName()
{
  return "geometry_msgs::msg::dds_::TwistStamped_";
}

template <> constexpr bool TopicTraits<::geometry_msgs::msg::dds_::TwistStamped_>::isSelfContained()
{
  return false;
}

template <> constexpr bool TopicTraits<::geometry_msgs::msg::dds_::TwistStamped_>::isKeyless()
{
  return true;
}

#ifdef DDSCXX_HAS_TYPE_DISCOVERY
template<> constexpr unsigned int TopicTraits<::geometry_msgs::msg::dds_::TwistStamped_>::type_map_blob_sz() { return 1378; }
template<> constexpr unsigned int TopicTraits<::geometry_msgs::msg::dds_::TwistStamped_>::type_info_blob_sz() { return 292; }
template<> inline const uint8_t * TopicTraits<::geometry_msgs::msg::dds_::TwistStamped_>::type_map_blob() {
  static const uint8_t blob[] = {
 0xc3,  0x01,  0x00,  0x00,  0x05,  0x00,  0x00,  0x00,  0xf1,  0x73,  0xe5,  0x49,  0xa3,  0xc7,  0xc7,  0x09, 
 0x40,  0xe3,  0xe7,  0xba,  0xfb,  0x9f,  0x5d,  0x00,  0x51,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x01,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x41,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00, 
 0x19,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf1,  0xdc,  0xf1,  0x2c,  0xd2,  0xdd, 
 0x5e,  0x71,  0x2c,  0xb7,  0xb1,  0xe5,  0x1f,  0xa3,  0xf2,  0x09,  0x9f,  0xb9,  0x95,  0x00,  0x00,  0x00, 
 0x19,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf1,  0x28,  0xcf,  0x21,  0x56,  0x4e, 
 0xbe,  0xe5,  0x84,  0x6f,  0x1b,  0x3c,  0xfb,  0x2b,  0x52,  0xeb,  0x3b,  0xac,  0x16,  0xf1,  0xdc,  0xf1, 
 0x2c,  0xd2,  0xdd,  0x5e,  0x71,  0x2c,  0xb7,  0xb1,  0xe5,  0x1f,  0xa3,  0xf2,  0x44,  0x00,  0x00,  0x00, 
 0xf1,  0x51,  0x01,  0x00,  0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x34,  0x00,  0x00,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x19,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf1,  0x56, 
 0x7c,  0x5a,  0x93,  0x54,  0x1c,  0x3b,  0x10,  0x86,  0xa4,  0xba,  0x46,  0xf9,  0x8d,  0x96,  0xb8,  0xc7, 
 0x8d,  0x00,  0x00,  0x00,  0x0c,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x70,  0x00, 
 0x4b,  0xb3,  0x9c,  0x5c,  0xf1,  0x56,  0x7c,  0x5a,  0x93,  0x54,  0x1c,  0x3b,  0x10,  0x86,  0xa4,  0xba, 
 0x46,  0xf9,  0x8d,  0x00,  0x33,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x01,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x23,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x0b,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x04,  0x74,  0x45,  0x9c,  0xa3,  0x00,  0x0b,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x07,  0xe2,  0x04,  0x64,  0xd5,  0xf1,  0x28,  0xcf,  0x21,  0x56, 
 0x4e,  0xbe,  0xe5,  0x84,  0x6f,  0x1b,  0x3c,  0xfb,  0x2b,  0x52,  0x00,  0x00,  0x51,  0x00,  0x00,  0x00, 
 0xf1,  0x51,  0x01,  0x00,  0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x41,  0x00,  0x00,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x19,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf1,  0x5e, 
 0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc,  0x9a,  0x93,  0x2b, 
 0x3c,  0x00,  0x00,  0x00,  0x19,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf1,  0x5e, 
 0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc,  0xd1,  0x8b,  0x86, 
 0x24,  0xf1,  0x5e,  0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc, 
 0x43,  0x00,  0x00,  0x00,  0xf1,  0x51,  0x01,  0x00,  0x01,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x33,  0x00,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00,  0x0b,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x0a,  0x9d,  0xd4,  0xe4,  0x61,  0x00,  0x0b,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x0a,  0x41,  0x52,  0x90,  0x76,  0x00,  0x0b,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x0a,  0xfb,  0xad,  0xe9,  0xe3,  0x00,  0xf8,  0x02,  0x00,  0x00,  0x05,  0x00,  0x00,  0x00, 
 0xf2,  0x47,  0x67,  0x84,  0x3d,  0x44,  0x90,  0x86,  0x2d,  0xa2,  0x41,  0xbe,  0x2e,  0x36,  0xde,  0x00, 
 0x94,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00,  0x30,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x28,  0x00,  0x00,  0x00,  0x67,  0x65,  0x6f,  0x6d,  0x65,  0x74,  0x72,  0x79,  0x5f,  0x6d,  0x73,  0x67, 
 0x73,  0x3a,  0x3a,  0x6d,  0x73,  0x67,  0x3a,  0x3a,  0x64,  0x64,  0x73,  0x5f,  0x3a,  0x3a,  0x54,  0x77, 
 0x69,  0x73,  0x74,  0x53,  0x74,  0x61,  0x6d,  0x70,  0x65,  0x64,  0x5f,  0x00,  0x58,  0x00,  0x00,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x25,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf2,  0xe5, 
 0x76,  0x5e,  0xc4,  0x8c,  0xff,  0xd4,  0x19,  0xed,  0x7f,  0xe8,  0x4e,  0x2a,  0x55,  0x00,  0x00,  0x00, 
 0x07,  0x00,  0x00,  0x00,  0x68,  0x65,  0x61,  0x64,  0x65,  0x72,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x24,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf2,  0x66,  0x9d,  0xb6,  0x2b,  0xd5, 
 0x53,  0x1f,  0x0b,  0xe5,  0x2c,  0x2a,  0x17,  0x6d,  0x6f,  0x00,  0x00,  0x00,  0x06,  0x00,  0x00,  0x00, 
 0x74,  0x77,  0x69,  0x73,  0x74,  0x00,  0x00,  0x00,  0xf2,  0xe5,  0x76,  0x5e,  0xc4,  0x8c,  0xff,  0xd4, 
 0x19,  0xed,  0x7f,  0xe8,  0x4e,  0x2a,  0x55,  0x00,  0x7b,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00, 
 0x25,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x1d,  0x00,  0x00,  0x00,  0x73,  0x74,  0x64,  0x5f, 
 0x6d,  0x73,  0x67,  0x73,  0x3a,  0x3a,  0x6d,  0x73,  0x67,  0x3a,  0x3a,  0x64,  0x64,  0x73,  0x5f,  0x3a, 
 0x3a,  0x48,  0x65,  0x61,  0x64,  0x65,  0x72,  0x5f,  0x00,  0x00,  0x00,  0x00,  0x47,  0x00,  0x00,  0x00, 
 0x02,  0x00,  0x00,  0x00,  0x24,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf2,  0xd4, 
 0x85,  0x4f,  0x13,  0xae,  0xf3,  0x2d,  0xfe,  0x21,  0x57,  0xf3,  0xe6,  0x32,  0x0d,  0x00,  0x00,  0x00, 
 0x06,  0x00,  0x00,  0x00,  0x73,  0x74,  0x61,  0x6d,  0x70,  0x00,  0x00,  0x00,  0x17,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x70,  0x00,  0x09,  0x00,  0x00,  0x00,  0x66,  0x72,  0x61,  0x6d, 
 0x65,  0x5f,  0x69,  0x64,  0x00,  0x00,  0x00,  0xf2,  0xd4,  0x85,  0x4f,  0x13,  0xae,  0xf3,  0x2d,  0xfe, 
 0x21,  0x57,  0xf3,  0xe6,  0x32,  0x0d,  0x00,  0x00,  0x72,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00, 
 0x2d,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x25,  0x00,  0x00,  0x00,  0x62,  0x75,  0x69,  0x6c, 
 0x74,  0x69,  0x6e,  0x5f,  0x69,  0x6e,  0x74,  0x65,  0x72,  0x66,  0x61,  0x63,  0x65,  0x73,  0x3a,  0x3a, 
 0x6d,  0x73,  0x67,  0x3a,  0x3a,  0x64,  0x64,  0x73,  0x5f,  0x3a,  0x3a,  0x54,  0x69,  0x6d,  0x65,  0x5f, 
 0x00,  0x00,  0x00,  0x00,  0x36,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x12,  0x00,  0x00,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0x04,  0x00,  0x04,  0x00,  0x00,  0x00,  0x73,  0x65,  0x63,  0x00, 
 0x00,  0x00,  0x00,  0x00,  0x16,  0x00,  0x00,  0x00,  0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x07,  0x00, 
 0x08,  0x00,  0x00,  0x00,  0x6e,  0x61,  0x6e,  0x6f,  0x73,  0x65,  0x63,  0x00,  0x00,  0x00,  0xf2,  0x66, 
 0x9d,  0xb6,  0x2b,  0xd5,  0x53,  0x1f,  0x0b,  0xe5,  0x2c,  0x2a,  0x17,  0x6d,  0x6f,  0x00,  0x00,  0x00, 
 0x92,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00,  0x29,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x21,  0x00,  0x00,  0x00,  0x67,  0x65,  0x6f,  0x6d,  0x65,  0x74,  0x72,  0x79,  0x5f,  0x6d,  0x73,  0x67, 
 0x73,  0x3a,  0x3a,  0x6d,  0x73,  0x67,  0x3a,  0x3a,  0x64,  0x64,  0x73,  0x5f,  0x3a,  0x3a,  0x54,  0x77, 
 0x69,  0x73,  0x74,  0x5f,  0x00,  0x00,  0x00,  0x00,  0x5a,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00, 
 0x25,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0, 
 0x1a,  0x7a,  0x3a,  0x5c,  0xda,  0xb3,  0x78,  0xfd,  0x32,  0x00,  0x00,  0x00,  0x07,  0x00,  0x00,  0x00, 
 0x6c,  0x69,  0x6e,  0x65,  0x61,  0x72,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x26,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c, 
 0xda,  0xb3,  0x78,  0xfd,  0x32,  0x00,  0x00,  0x00,  0x08,  0x00,  0x00,  0x00,  0x61,  0x6e,  0x67,  0x75, 
 0x6c,  0x61,  0x72,  0x00,  0x00,  0x00,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c, 
 0xda,  0xb3,  0x78,  0xfd,  0x32,  0x00,  0x00,  0x00,  0x78,  0x00,  0x00,  0x00,  0xf2,  0x51,  0x01,  0x00, 
 0x2b,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x23,  0x00,  0x00,  0x00,  0x67,  0x65,  0x6f,  0x6d, 
 0x65,  0x74,  0x72,  0x79,  0x5f,  0x6d,  0x73,  0x67,  0x73,  0x3a,  0x3a,  0x6d,  0x73,  0x67,  0x3a,  0x3a, 
 0x64,  0x64,  0x73,  0x5f,  0x3a,  0x3a,  0x56,  0x65,  0x63,  0x74,  0x6f,  0x72,  0x33,  0x5f,  0x00,  0x00, 
 0x40,  0x00,  0x00,  0x00,  0x03,  0x00,  0x00,  0x00,  0x10,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x0a,  0x00,  0x02,  0x00,  0x00,  0x00,  0x78,  0x00,  0x00,  0x00,  0x10,  0x00,  0x00,  0x00, 
 0x01,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x00,  0x02,  0x00,  0x00,  0x00,  0x79,  0x00,  0x00,  0x00, 
 0x10,  0x00,  0x00,  0x00,  0x02,  0x00,  0x00,  0x00,  0x01,  0x00,  0x0a,  0x00,  0x02,  0x00,  0x00,  0x00, 
 0x7a,  0x00,  0x00,  0x00,  0x9a,  0x00,  0x00,  0x00,  0x05,  0x00,  0x00,  0x00,  0xf2,  0x47,  0x67,  0x84, 
 0x3d,  0x44,  0x90,  0x86,  0x2d,  0xa2,  0x41,  0xbe,  0x2e,  0x36,  0xde,  0xf1,  0x73,  0xe5,  0x49,  0xa3, 
 0xc7,  0xc7,  0x09,  0x40,  0xe3,  0xe7,  0xba,  0xfb,  0x9f,  0x5d,  0xf2,  0xe5,  0x76,  0x5e,  0xc4,  0x8c, 
 0xff,  0xd4,  0x19,  0xed,  0x7f,  0xe8,  0x4e,  0x2a,  0x55,  0xf1,  0xdc,  0xf1,  0x2c,  0xd2,  0xdd,  0x5e, 
 0x71,  0x2c,  0xb7,  0xb1,  0xe5,  0x1f,  0xa3,  0xf2,  0xf2,  0xd4,  0x85,  0x4f,  0x13,  0xae,  0xf3,  0x2d, 
 0xfe,  0x21,  0x57,  0xf3,  0xe6,  0x32,  0x0d,  0xf1,  0x56,  0x7c,  0x5a,  0x93,  0x54,  0x1c,  0x3b,  0x10, 
 0x86,  0xa4,  0xba,  0x46,  0xf9,  0x8d,  0xf2,  0x66,  0x9d,  0xb6,  0x2b,  0xd5,  0x53,  0x1f,  0x0b,  0xe5, 
 0x2c,  0x2a,  0x17,  0x6d,  0x6f,  0xf1,  0x28,  0xcf,  0x21,  0x56,  0x4e,  0xbe,  0xe5,  0x84,  0x6f,  0x1b, 
 0x3c,  0xfb,  0x2b,  0x52,  0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c,  0xda,  0xb3, 
 0x78,  0xfd,  0x32,  0xf1,  0x5e,  0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd, 
 0x4c,  0xbc, };
  return blob;
}
template<> inline const uint8_t * TopicTraits<::geometry_msgs::msg::dds_::TwistStamped_>::type_info_blob() {
  static const uint8_t blob[] = {
 0x20,  0x01,  0x00,  0x00,  0x01,  0x10,  0x00,  0x40,  0x88,  0x00,  0x00,  0x00,  0x84,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf1,  0x73,  0xe5,  0x49,  0xa3,  0xc7,  0xc7,  0x09,  0x40,  0xe3,  0xe7,  0xba, 
 0xfb,  0x9f,  0x5d,  0x00,  0x55,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00,  0x64,  0x00,  0x00,  0x00, 
 0x04,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0xf1,  0xdc,  0xf1,  0x2c,  0xd2,  0xdd,  0x5e,  0x71, 
 0x2c,  0xb7,  0xb1,  0xe5,  0x1f,  0xa3,  0xf2,  0x00,  0x48,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00, 
 0xf1,  0x56,  0x7c,  0x5a,  0x93,  0x54,  0x1c,  0x3b,  0x10,  0x86,  0xa4,  0xba,  0x46,  0xf9,  0x8d,  0x00, 
 0x37,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0xf1,  0x28,  0xcf,  0x21,  0x56,  0x4e,  0xbe,  0xe5, 
 0x84,  0x6f,  0x1b,  0x3c,  0xfb,  0x2b,  0x52,  0x00,  0x55,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00, 
 0xf1,  0x5e,  0x73,  0x97,  0xe7,  0xe8,  0x64,  0x40,  0xdf,  0x64,  0xaf,  0x76,  0xcd,  0x4c,  0xbc,  0x00, 
 0x47,  0x00,  0x00,  0x00,  0x02,  0x10,  0x00,  0x40,  0x88,  0x00,  0x00,  0x00,  0x84,  0x00,  0x00,  0x00, 
 0x14,  0x00,  0x00,  0x00,  0xf2,  0x47,  0x67,  0x84,  0x3d,  0x44,  0x90,  0x86,  0x2d,  0xa2,  0x41,  0xbe, 
 0x2e,  0x36,  0xde,  0x00,  0x98,  0x00,  0x00,  0x00,  0x04,  0x00,  0x00,  0x00,  0x64,  0x00,  0x00,  0x00, 
 0x04,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0xf2,  0xe5,  0x76,  0x5e,  0xc4,  0x8c,  0xff,  0xd4, 
 0x19,  0xed,  0x7f,  0xe8,  0x4e,  0x2a,  0x55,  0x00,  0x7f,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00, 
 0xf2,  0xd4,  0x85,  0x4f,  0x13,  0xae,  0xf3,  0x2d,  0xfe,  0x21,  0x57,  0xf3,  0xe6,  0x32,  0x0d,  0x00, 
 0x76,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00,  0xf2,  0x66,  0x9d,  0xb6,  0x2b,  0xd5,  0x53,  0x1f, 
 0x0b,  0xe5,  0x2c,  0x2a,  0x17,  0x6d,  0x6f,  0x00,  0x96,  0x00,  0x00,  0x00,  0x14,  0x00,  0x00,  0x00, 
 0xf2,  0xac,  0xe2,  0x5e,  0x59,  0xb0,  0x1a,  0x7a,  0x3a,  0x5c,  0xda,  0xb3,  0x78,  0xfd,  0x32,  0x00, 
 0x7c,  0x00,  0x00,  0x00, };
  return blob;
}
#endif //DDSCXX_HAS_TYPE_DISCOVERY

} //namespace topic
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

namespace dds {
namespace topic {

template <>
struct topic_type_name<::geometry_msgs::msg::dds_::TwistStamped_>
{
    static std::string value()
    {
      return org::eclipse::cyclonedds::topic::TopicTraits<::geometry_msgs::msg::dds_::TwistStamped_>::getTypeName();
    }
};

}
}

REGISTER_TOPIC_TYPE(::geometry_msgs::msg::dds_::TwistStamped_)

namespace org{
namespace eclipse{
namespace cyclonedds{
namespace core{
namespace cdr{

template<>
propvec &get_type_props<::geometry_msgs::msg::dds_::TwistStamped_>();

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool write(T& streamer, const ::geometry_msgs::msg::dds_::TwistStamped_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.header(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!write(streamer, instance.twist(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool write(S& str, const ::geometry_msgs::msg::dds_::TwistStamped_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::TwistStamped_>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool read(T& streamer, ::geometry_msgs::msg::dds_::TwistStamped_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.header(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!read(streamer, instance.twist(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool read(S& str, ::geometry_msgs::msg::dds_::TwistStamped_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::TwistStamped_>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool move(T& streamer, const ::geometry_msgs::msg::dds_::TwistStamped_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.header(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!move(streamer, instance.twist(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool move(S& str, const ::geometry_msgs::msg::dds_::TwistStamped_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::TwistStamped_>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props.data()); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool max(T& streamer, const ::geometry_msgs::msg::dds_::TwistStamped_& instance, entity_properties_t *props) {
  (void)instance;
  if (!streamer.start_struct(*props))
    return false;
  auto prop = streamer.first_entity(props);
  while (prop) {
    switch (prop->m_id) {
      case 0:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.header(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(*prop))
        return false;
      if (!max(streamer, instance.twist(), prop))
        return false;
      if (!streamer.finish_member(*prop))
        return false;
      break;
    }
    prop = streamer.next_entity(prop);
  }
  return streamer.finish_struct(*props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool max(S& str, const ::geometry_msgs::msg::dds_::TwistStamped_& instance, bool as_key) {
  auto &props = get_type_props<::geometry_msgs::msg::dds_::TwistStamped_>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props.data()); 
}

} //namespace cdr
} //namespace core
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

#endif // DDSCXX_UNITREE_IDL_ROS2_TWISTSTAMPED__HPP
