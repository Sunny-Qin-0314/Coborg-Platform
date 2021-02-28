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

class OffsetPlayback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.offset = null;
    }
    else {
      if (initObj.hasOwnProperty('offset')) {
        this.offset = initObj.offset
      }
      else {
        this.offset = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OffsetPlayback
    // Serialize message field [offset]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.offset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OffsetPlayback
    let len;
    let data = new OffsetPlayback(null);
    // Deserialize message field [offset]
    data.offset = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hebi_cpp_api_examples/OffsetPlayback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'de19cca9344eb5bfedb7e55986a47f2e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point offset
    
    
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
    const resolved = new OffsetPlayback(null);
    if (msg.offset !== undefined) {
      resolved.offset = geometry_msgs.msg.Point.Resolve(msg.offset)
    }
    else {
      resolved.offset = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = OffsetPlayback;
