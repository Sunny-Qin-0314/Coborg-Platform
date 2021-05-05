// Auto-generated. Do not edit!

// (in-package franka_interface_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ExecuteSkillFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.num_execution_feedback = null;
      this.execution_feedback = null;
    }
    else {
      if (initObj.hasOwnProperty('num_execution_feedback')) {
        this.num_execution_feedback = initObj.num_execution_feedback
      }
      else {
        this.num_execution_feedback = 0;
      }
      if (initObj.hasOwnProperty('execution_feedback')) {
        this.execution_feedback = initObj.execution_feedback
      }
      else {
        this.execution_feedback = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExecuteSkillFeedback
    // Serialize message field [num_execution_feedback]
    bufferOffset = _serializer.uint64(obj.num_execution_feedback, buffer, bufferOffset);
    // Serialize message field [execution_feedback]
    bufferOffset = _arraySerializer.uint8(obj.execution_feedback, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExecuteSkillFeedback
    let len;
    let data = new ExecuteSkillFeedback(null);
    // Deserialize message field [num_execution_feedback]
    data.num_execution_feedback = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [execution_feedback]
    data.execution_feedback = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.execution_feedback.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'franka_interface_msgs/ExecuteSkillFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ff908ea97d128e4dfc430213d4a485e0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    # Feedback message - happens during skill execution 
    uint64 num_execution_feedback
    uint8[] execution_feedback
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExecuteSkillFeedback(null);
    if (msg.num_execution_feedback !== undefined) {
      resolved.num_execution_feedback = msg.num_execution_feedback;
    }
    else {
      resolved.num_execution_feedback = 0
    }

    if (msg.execution_feedback !== undefined) {
      resolved.execution_feedback = msg.execution_feedback;
    }
    else {
      resolved.execution_feedback = []
    }

    return resolved;
    }
};

module.exports = ExecuteSkillFeedback;