// Auto-generated. Do not edit!

// (in-package hebi_cpp_api_examples.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetFeedbackFrequencyRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.frequency_hz = null;
    }
    else {
      if (initObj.hasOwnProperty('frequency_hz')) {
        this.frequency_hz = initObj.frequency_hz
      }
      else {
        this.frequency_hz = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetFeedbackFrequencyRequest
    // Serialize message field [frequency_hz]
    bufferOffset = _serializer.float64(obj.frequency_hz, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetFeedbackFrequencyRequest
    let len;
    let data = new SetFeedbackFrequencyRequest(null);
    // Deserialize message field [frequency_hz]
    data.frequency_hz = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebi_cpp_api_examples/SetFeedbackFrequencyRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b11146fd2143e78325a7496114ee3a9e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 frequency_hz
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetFeedbackFrequencyRequest(null);
    if (msg.frequency_hz !== undefined) {
      resolved.frequency_hz = msg.frequency_hz;
    }
    else {
      resolved.frequency_hz = 0.0
    }

    return resolved;
    }
};

class SetFeedbackFrequencyResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetFeedbackFrequencyResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetFeedbackFrequencyResponse
    let len;
    let data = new SetFeedbackFrequencyResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebi_cpp_api_examples/SetFeedbackFrequencyResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetFeedbackFrequencyResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetFeedbackFrequencyRequest,
  Response: SetFeedbackFrequencyResponse,
  md5sum() { return 'b11146fd2143e78325a7496114ee3a9e'; },
  datatype() { return 'hebi_cpp_api_examples/SetFeedbackFrequency'; }
};
