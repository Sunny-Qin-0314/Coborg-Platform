// Auto-generated. Do not edit!

// (in-package rosserial_arduino.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CMU {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Potentiometer = null;
      this.Flex_Sensor = null;
      this.IR_Sensor = null;
      this.Ultrasonic_Sensor = null;
      this.Button_State = null;
    }
    else {
      if (initObj.hasOwnProperty('Potentiometer')) {
        this.Potentiometer = initObj.Potentiometer
      }
      else {
        this.Potentiometer = 0;
      }
      if (initObj.hasOwnProperty('Flex_Sensor')) {
        this.Flex_Sensor = initObj.Flex_Sensor
      }
      else {
        this.Flex_Sensor = 0;
      }
      if (initObj.hasOwnProperty('IR_Sensor')) {
        this.IR_Sensor = initObj.IR_Sensor
      }
      else {
        this.IR_Sensor = 0;
      }
      if (initObj.hasOwnProperty('Ultrasonic_Sensor')) {
        this.Ultrasonic_Sensor = initObj.Ultrasonic_Sensor
      }
      else {
        this.Ultrasonic_Sensor = 0;
      }
      if (initObj.hasOwnProperty('Button_State')) {
        this.Button_State = initObj.Button_State
      }
      else {
        this.Button_State = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CMU
    // Serialize message field [Potentiometer]
    bufferOffset = _serializer.uint16(obj.Potentiometer, buffer, bufferOffset);
    // Serialize message field [Flex_Sensor]
    bufferOffset = _serializer.uint16(obj.Flex_Sensor, buffer, bufferOffset);
    // Serialize message field [IR_Sensor]
    bufferOffset = _serializer.uint16(obj.IR_Sensor, buffer, bufferOffset);
    // Serialize message field [Ultrasonic_Sensor]
    bufferOffset = _serializer.uint16(obj.Ultrasonic_Sensor, buffer, bufferOffset);
    // Serialize message field [Button_State]
    bufferOffset = _serializer.uint16(obj.Button_State, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CMU
    let len;
    let data = new CMU(null);
    // Deserialize message field [Potentiometer]
    data.Potentiometer = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [Flex_Sensor]
    data.Flex_Sensor = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [IR_Sensor]
    data.IR_Sensor = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [Ultrasonic_Sensor]
    data.Ultrasonic_Sensor = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [Button_State]
    data.Button_State = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rosserial_arduino/CMU';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'be9cdfeec1c60f4d26327be218060167';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 Potentiometer
    uint16 Flex_Sensor
    uint16 IR_Sensor
    uint16 Ultrasonic_Sensor
    uint16 Button_State
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CMU(null);
    if (msg.Potentiometer !== undefined) {
      resolved.Potentiometer = msg.Potentiometer;
    }
    else {
      resolved.Potentiometer = 0
    }

    if (msg.Flex_Sensor !== undefined) {
      resolved.Flex_Sensor = msg.Flex_Sensor;
    }
    else {
      resolved.Flex_Sensor = 0
    }

    if (msg.IR_Sensor !== undefined) {
      resolved.IR_Sensor = msg.IR_Sensor;
    }
    else {
      resolved.IR_Sensor = 0
    }

    if (msg.Ultrasonic_Sensor !== undefined) {
      resolved.Ultrasonic_Sensor = msg.Ultrasonic_Sensor;
    }
    else {
      resolved.Ultrasonic_Sensor = 0
    }

    if (msg.Button_State !== undefined) {
      resolved.Button_State = msg.Button_State;
    }
    else {
      resolved.Button_State = 0
    }

    return resolved;
    }
};

module.exports = CMU;
