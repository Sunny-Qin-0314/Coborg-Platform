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

class SetGainsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.gains_package = null;
      this.gains_file = null;
    }
    else {
      if (initObj.hasOwnProperty('gains_package')) {
        this.gains_package = initObj.gains_package
      }
      else {
        this.gains_package = '';
      }
      if (initObj.hasOwnProperty('gains_file')) {
        this.gains_file = initObj.gains_file
      }
      else {
        this.gains_file = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetGainsRequest
    // Serialize message field [gains_package]
    bufferOffset = _serializer.string(obj.gains_package, buffer, bufferOffset);
    // Serialize message field [gains_file]
    bufferOffset = _serializer.string(obj.gains_file, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetGainsRequest
    let len;
    let data = new SetGainsRequest(null);
    // Deserialize message field [gains_package]
    data.gains_package = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [gains_file]
    data.gains_file = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.gains_package.length;
    length += object.gains_file.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebi_cpp_api_examples/SetGainsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4fedf26e82cd5de1e91a4ab742d58b2c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string gains_package
    string gains_file
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetGainsRequest(null);
    if (msg.gains_package !== undefined) {
      resolved.gains_package = msg.gains_package;
    }
    else {
      resolved.gains_package = ''
    }

    if (msg.gains_file !== undefined) {
      resolved.gains_file = msg.gains_file;
    }
    else {
      resolved.gains_file = ''
    }

    return resolved;
    }
};

class SetGainsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetGainsResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetGainsResponse
    let len;
    let data = new SetGainsResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebi_cpp_api_examples/SetGainsResponse';
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
    const resolved = new SetGainsResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetGainsRequest,
  Response: SetGainsResponse,
  md5sum() { return '4fedf26e82cd5de1e91a4ab742d58b2c'; },
  datatype() { return 'hebi_cpp_api_examples/SetGains'; }
};
