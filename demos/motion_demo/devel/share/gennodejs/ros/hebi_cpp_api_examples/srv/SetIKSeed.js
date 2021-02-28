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

class SetIKSeedRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.seed = null;
    }
    else {
      if (initObj.hasOwnProperty('seed')) {
        this.seed = initObj.seed
      }
      else {
        this.seed = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetIKSeedRequest
    // Serialize message field [seed]
    bufferOffset = _arraySerializer.float64(obj.seed, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIKSeedRequest
    let len;
    let data = new SetIKSeedRequest(null);
    // Deserialize message field [seed]
    data.seed = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.seed.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebi_cpp_api_examples/SetIKSeedRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '45f70114af10b3f5f5e2b664d72ca331';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] seed
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetIKSeedRequest(null);
    if (msg.seed !== undefined) {
      resolved.seed = msg.seed;
    }
    else {
      resolved.seed = []
    }

    return resolved;
    }
};

class SetIKSeedResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetIKSeedResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIKSeedResponse
    let len;
    let data = new SetIKSeedResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'hebi_cpp_api_examples/SetIKSeedResponse';
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
    const resolved = new SetIKSeedResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: SetIKSeedRequest,
  Response: SetIKSeedResponse,
  md5sum() { return '45f70114af10b3f5f5e2b664d72ca331'; },
  datatype() { return 'hebi_cpp_api_examples/SetIKSeed'; }
};
