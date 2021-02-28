// Auto-generated. Do not edit!

// (in-package hebi_cpp_api_examples.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class TargetWaypoints {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.waypoints_vector = null;
    }
    else {
      if (initObj.hasOwnProperty('waypoints_vector')) {
        this.waypoints_vector = initObj.waypoints_vector
      }
      else {
        this.waypoints_vector = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TargetWaypoints
    // Serialize message field [waypoints_vector]
    // Serialize the length for message field [waypoints_vector]
    bufferOffset = _serializer.uint32(obj.waypoints_vector.length, buffer, bufferOffset);
    obj.waypoints_vector.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TargetWaypoints
    let len;
    let data = new TargetWaypoints(null);
    // Deserialize message field [waypoints_vector]
    // Deserialize array length for message field [waypoints_vector]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.waypoints_vector = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.waypoints_vector[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.waypoints_vector.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hebi_cpp_api_examples/TargetWaypoints';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa0b6e1fb814653675dd4da6b4a51d95';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point[] waypoints_vector
    
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TargetWaypoints(null);
    if (msg.waypoints_vector !== undefined) {
      resolved.waypoints_vector = new Array(msg.waypoints_vector.length);
      for (let i = 0; i < resolved.waypoints_vector.length; ++i) {
        resolved.waypoints_vector[i] = geometry_msgs.msg.Point.Resolve(msg.waypoints_vector[i]);
      }
    }
    else {
      resolved.waypoints_vector = []
    }

    return resolved;
    }
};

module.exports = TargetWaypoints;
