// Auto-generated. Do not edit!

// (in-package hebi_cpp_api_examples.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ArmMotionGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.tipx = null;
      this.tipy = null;
      this.tipz = null;
      this.set_color = null;
      this.r = null;
      this.g = null;
      this.b = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = [];
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = [];
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = [];
      }
      if (initObj.hasOwnProperty('tipx')) {
        this.tipx = initObj.tipx
      }
      else {
        this.tipx = [];
      }
      if (initObj.hasOwnProperty('tipy')) {
        this.tipy = initObj.tipy
      }
      else {
        this.tipy = [];
      }
      if (initObj.hasOwnProperty('tipz')) {
        this.tipz = initObj.tipz
      }
      else {
        this.tipz = [];
      }
      if (initObj.hasOwnProperty('set_color')) {
        this.set_color = initObj.set_color
      }
      else {
        this.set_color = false;
      }
      if (initObj.hasOwnProperty('r')) {
        this.r = initObj.r
      }
      else {
        this.r = 0;
      }
      if (initObj.hasOwnProperty('g')) {
        this.g = initObj.g
      }
      else {
        this.g = 0;
      }
      if (initObj.hasOwnProperty('b')) {
        this.b = initObj.b
      }
      else {
        this.b = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmMotionGoal
    // Serialize message field [x]
    bufferOffset = _arraySerializer.float64(obj.x, buffer, bufferOffset, null);
    // Serialize message field [y]
    bufferOffset = _arraySerializer.float64(obj.y, buffer, bufferOffset, null);
    // Serialize message field [z]
    bufferOffset = _arraySerializer.float64(obj.z, buffer, bufferOffset, null);
    // Serialize message field [tipx]
    bufferOffset = _arraySerializer.float64(obj.tipx, buffer, bufferOffset, null);
    // Serialize message field [tipy]
    bufferOffset = _arraySerializer.float64(obj.tipy, buffer, bufferOffset, null);
    // Serialize message field [tipz]
    bufferOffset = _arraySerializer.float64(obj.tipz, buffer, bufferOffset, null);
    // Serialize message field [set_color]
    bufferOffset = _serializer.bool(obj.set_color, buffer, bufferOffset);
    // Serialize message field [r]
    bufferOffset = _serializer.uint8(obj.r, buffer, bufferOffset);
    // Serialize message field [g]
    bufferOffset = _serializer.uint8(obj.g, buffer, bufferOffset);
    // Serialize message field [b]
    bufferOffset = _serializer.uint8(obj.b, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmMotionGoal
    let len;
    let data = new ArmMotionGoal(null);
    // Deserialize message field [x]
    data.x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [y]
    data.y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [z]
    data.z = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [tipx]
    data.tipx = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [tipy]
    data.tipy = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [tipz]
    data.tipz = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [set_color]
    data.set_color = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [r]
    data.r = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [g]
    data.g = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [b]
    data.b = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.x.length;
    length += 8 * object.y.length;
    length += 8 * object.z.length;
    length += 8 * object.tipx.length;
    length += 8 * object.tipy.length;
    length += 8 * object.tipz.length;
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'hebi_cpp_api_examples/ArmMotionGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5182f7e080f47b29c479712b9f962cb9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    # Goal position:
    float64[] x
    float64[] y
    float64[] z
    float64[] tipx
    float64[] tipy
    float64[] tipz
    
    # Optionally, set a color when doing the move; otherwise, clear the color.
    bool set_color
    uint8 r
    uint8 g
    uint8 b
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ArmMotionGoal(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = []
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = []
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = []
    }

    if (msg.tipx !== undefined) {
      resolved.tipx = msg.tipx;
    }
    else {
      resolved.tipx = []
    }

    if (msg.tipy !== undefined) {
      resolved.tipy = msg.tipy;
    }
    else {
      resolved.tipy = []
    }

    if (msg.tipz !== undefined) {
      resolved.tipz = msg.tipz;
    }
    else {
      resolved.tipz = []
    }

    if (msg.set_color !== undefined) {
      resolved.set_color = msg.set_color;
    }
    else {
      resolved.set_color = false
    }

    if (msg.r !== undefined) {
      resolved.r = msg.r;
    }
    else {
      resolved.r = 0
    }

    if (msg.g !== undefined) {
      resolved.g = msg.g;
    }
    else {
      resolved.g = 0
    }

    if (msg.b !== undefined) {
      resolved.b = msg.b;
    }
    else {
      resolved.b = 0
    }

    return resolved;
    }
};

module.exports = ArmMotionGoal;
