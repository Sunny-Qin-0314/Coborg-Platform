// Auto-generated. Do not edit!

// (in-package ros_detect_planes_from_depth_img.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PlanesResults {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.N = null;
      this.norms = null;
      this.center_3d = null;
      this.center_2d = null;
      this.mask_color = null;
    }
    else {
      if (initObj.hasOwnProperty('N')) {
        this.N = initObj.N
      }
      else {
        this.N = 0;
      }
      if (initObj.hasOwnProperty('norms')) {
        this.norms = initObj.norms
      }
      else {
        this.norms = [];
      }
      if (initObj.hasOwnProperty('center_3d')) {
        this.center_3d = initObj.center_3d
      }
      else {
        this.center_3d = [];
      }
      if (initObj.hasOwnProperty('center_2d')) {
        this.center_2d = initObj.center_2d
      }
      else {
        this.center_2d = [];
      }
      if (initObj.hasOwnProperty('mask_color')) {
        this.mask_color = initObj.mask_color
      }
      else {
        this.mask_color = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanesResults
    // Serialize message field [N]
    bufferOffset = _serializer.int32(obj.N, buffer, bufferOffset);
    // Serialize message field [norms]
    bufferOffset = _arraySerializer.float32(obj.norms, buffer, bufferOffset, null);
    // Serialize message field [center_3d]
    bufferOffset = _arraySerializer.float32(obj.center_3d, buffer, bufferOffset, null);
    // Serialize message field [center_2d]
    bufferOffset = _arraySerializer.float32(obj.center_2d, buffer, bufferOffset, null);
    // Serialize message field [mask_color]
    bufferOffset = _arraySerializer.uint16(obj.mask_color, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanesResults
    let len;
    let data = new PlanesResults(null);
    // Deserialize message field [N]
    data.N = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [norms]
    data.norms = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [center_3d]
    data.center_3d = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [center_2d]
    data.center_2d = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [mask_color]
    data.mask_color = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.norms.length;
    length += 4 * object.center_3d.length;
    length += 4 * object.center_2d.length;
    length += 2 * object.mask_color.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_detect_planes_from_depth_img/PlanesResults';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd1490bcb974cae216e975f12f5d851b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 N                 # Number of detected planes.
    
    # In the following arrays, 
    # the planes' parameters are concatinated one by one.
    
    float32[] norms         # Nx3. Plane normal (nx, ny, nz).
    float32[] center_3d     # Nx3. Plane 3D center (cx, cy, cz).
    float32[] center_2d     # Nx2. Plane 2D center on the image (px, py),
                            #   which means {px}th column, and {py}th row.
    uint16[] mask_color     # Nx3. Plane mask color (blue, green, red). 
                            # Each color's range is [0, 255]
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanesResults(null);
    if (msg.N !== undefined) {
      resolved.N = msg.N;
    }
    else {
      resolved.N = 0
    }

    if (msg.norms !== undefined) {
      resolved.norms = msg.norms;
    }
    else {
      resolved.norms = []
    }

    if (msg.center_3d !== undefined) {
      resolved.center_3d = msg.center_3d;
    }
    else {
      resolved.center_3d = []
    }

    if (msg.center_2d !== undefined) {
      resolved.center_2d = msg.center_2d;
    }
    else {
      resolved.center_2d = []
    }

    if (msg.mask_color !== undefined) {
      resolved.mask_color = msg.mask_color;
    }
    else {
      resolved.mask_color = []
    }

    return resolved;
    }
};

module.exports = PlanesResults;
