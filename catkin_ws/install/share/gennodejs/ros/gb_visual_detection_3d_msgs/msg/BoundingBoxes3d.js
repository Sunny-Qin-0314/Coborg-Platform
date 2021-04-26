// Auto-generated. Do not edit!

// (in-package gb_visual_detection_3d_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BoundingBox3d = require('./BoundingBox3d.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class BoundingBoxes3d {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.bounding_boxes = null;
      this.normal_x = null;
      this.normal_y = null;
      this.normal_z = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('bounding_boxes')) {
        this.bounding_boxes = initObj.bounding_boxes
      }
      else {
        this.bounding_boxes = [];
      }
      if (initObj.hasOwnProperty('normal_x')) {
        this.normal_x = initObj.normal_x
      }
      else {
        this.normal_x = 0.0;
      }
      if (initObj.hasOwnProperty('normal_y')) {
        this.normal_y = initObj.normal_y
      }
      else {
        this.normal_y = 0.0;
      }
      if (initObj.hasOwnProperty('normal_z')) {
        this.normal_z = initObj.normal_z
      }
      else {
        this.normal_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoundingBoxes3d
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [bounding_boxes]
    // Serialize the length for message field [bounding_boxes]
    bufferOffset = _serializer.uint32(obj.bounding_boxes.length, buffer, bufferOffset);
    obj.bounding_boxes.forEach((val) => {
      bufferOffset = BoundingBox3d.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [normal_x]
    bufferOffset = _serializer.float64(obj.normal_x, buffer, bufferOffset);
    // Serialize message field [normal_y]
    bufferOffset = _serializer.float64(obj.normal_y, buffer, bufferOffset);
    // Serialize message field [normal_z]
    bufferOffset = _serializer.float64(obj.normal_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoundingBoxes3d
    let len;
    let data = new BoundingBoxes3d(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [bounding_boxes]
    // Deserialize array length for message field [bounding_boxes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.bounding_boxes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.bounding_boxes[i] = BoundingBox3d.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [normal_x]
    data.normal_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [normal_y]
    data.normal_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [normal_z]
    data.normal_z = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.bounding_boxes.forEach((val) => {
      length += BoundingBox3d.getMessageSize(val);
    });
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gb_visual_detection_3d_msgs/BoundingBoxes3d';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1fb42483703ab1406c7e101f990caf57';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    BoundingBox3d[] bounding_boxes
    float64 normal_x
    float64 normal_y
    float64 normal_z
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: gb_visual_detection_3d_msgs/BoundingBox3d
    string Class
    float64 probability
    float64 xmin
    float64 ymin
    float64 xmax
    float64 ymax
    float64 zmin
    float64 zmax
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoundingBoxes3d(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.bounding_boxes !== undefined) {
      resolved.bounding_boxes = new Array(msg.bounding_boxes.length);
      for (let i = 0; i < resolved.bounding_boxes.length; ++i) {
        resolved.bounding_boxes[i] = BoundingBox3d.Resolve(msg.bounding_boxes[i]);
      }
    }
    else {
      resolved.bounding_boxes = []
    }

    if (msg.normal_x !== undefined) {
      resolved.normal_x = msg.normal_x;
    }
    else {
      resolved.normal_x = 0.0
    }

    if (msg.normal_y !== undefined) {
      resolved.normal_y = msg.normal_y;
    }
    else {
      resolved.normal_y = 0.0
    }

    if (msg.normal_z !== undefined) {
      resolved.normal_z = msg.normal_z;
    }
    else {
      resolved.normal_z = 0.0
    }

    return resolved;
    }
};

module.exports = BoundingBoxes3d;
