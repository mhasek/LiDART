// Auto-generated. Do not edit!

// (in-package lidart_gap_finding.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class gaps {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.r1 = null;
      this.theta1 = null;
      this.r2 = null;
      this.theta2 = null;
      this.x1 = null;
      this.y1 = null;
      this.x2 = null;
      this.y2 = null;
      this.euc_length = null;
      this.delta_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('r1')) {
        this.r1 = initObj.r1
      }
      else {
        this.r1 = [];
      }
      if (initObj.hasOwnProperty('theta1')) {
        this.theta1 = initObj.theta1
      }
      else {
        this.theta1 = [];
      }
      if (initObj.hasOwnProperty('r2')) {
        this.r2 = initObj.r2
      }
      else {
        this.r2 = [];
      }
      if (initObj.hasOwnProperty('theta2')) {
        this.theta2 = initObj.theta2
      }
      else {
        this.theta2 = [];
      }
      if (initObj.hasOwnProperty('x1')) {
        this.x1 = initObj.x1
      }
      else {
        this.x1 = [];
      }
      if (initObj.hasOwnProperty('y1')) {
        this.y1 = initObj.y1
      }
      else {
        this.y1 = [];
      }
      if (initObj.hasOwnProperty('x2')) {
        this.x2 = initObj.x2
      }
      else {
        this.x2 = [];
      }
      if (initObj.hasOwnProperty('y2')) {
        this.y2 = initObj.y2
      }
      else {
        this.y2 = [];
      }
      if (initObj.hasOwnProperty('euc_length')) {
        this.euc_length = initObj.euc_length
      }
      else {
        this.euc_length = [];
      }
      if (initObj.hasOwnProperty('delta_angle')) {
        this.delta_angle = initObj.delta_angle
      }
      else {
        this.delta_angle = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type gaps
    // Serialize message field [r1]
    bufferOffset = _arraySerializer.float32(obj.r1, buffer, bufferOffset, null);
    // Serialize message field [theta1]
    bufferOffset = _arraySerializer.float32(obj.theta1, buffer, bufferOffset, null);
    // Serialize message field [r2]
    bufferOffset = _arraySerializer.float32(obj.r2, buffer, bufferOffset, null);
    // Serialize message field [theta2]
    bufferOffset = _arraySerializer.float32(obj.theta2, buffer, bufferOffset, null);
    // Serialize message field [x1]
    bufferOffset = _arraySerializer.float32(obj.x1, buffer, bufferOffset, null);
    // Serialize message field [y1]
    bufferOffset = _arraySerializer.float32(obj.y1, buffer, bufferOffset, null);
    // Serialize message field [x2]
    bufferOffset = _arraySerializer.float32(obj.x2, buffer, bufferOffset, null);
    // Serialize message field [y2]
    bufferOffset = _arraySerializer.float32(obj.y2, buffer, bufferOffset, null);
    // Serialize message field [euc_length]
    bufferOffset = _arraySerializer.float32(obj.euc_length, buffer, bufferOffset, null);
    // Serialize message field [delta_angle]
    bufferOffset = _arraySerializer.float32(obj.delta_angle, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type gaps
    let len;
    let data = new gaps(null);
    // Deserialize message field [r1]
    data.r1 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [theta1]
    data.theta1 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [r2]
    data.r2 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [theta2]
    data.theta2 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [x1]
    data.x1 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [y1]
    data.y1 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [x2]
    data.x2 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [y2]
    data.y2 = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [euc_length]
    data.euc_length = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [delta_angle]
    data.delta_angle = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.r1.length;
    length += 4 * object.theta1.length;
    length += 4 * object.r2.length;
    length += 4 * object.theta2.length;
    length += 4 * object.x1.length;
    length += 4 * object.y1.length;
    length += 4 * object.x2.length;
    length += 4 * object.y2.length;
    length += 4 * object.euc_length.length;
    length += 4 * object.delta_angle.length;
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidart_gap_finding/gaps';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c57a067c5b046afa4cdb6b94f79d87d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # angles are angle from lidar scan
    # ranges are distance from lidar
    # lengths are lengths of gap
    
    float32[] r1
    float32[] theta1
    float32[] r2
    float32[] theta2
    
    float32[] x1
    float32[] y1
    float32[] x2
    float32[] y2
    
    float32[] euc_length
    float32[] delta_angle
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new gaps(null);
    if (msg.r1 !== undefined) {
      resolved.r1 = msg.r1;
    }
    else {
      resolved.r1 = []
    }

    if (msg.theta1 !== undefined) {
      resolved.theta1 = msg.theta1;
    }
    else {
      resolved.theta1 = []
    }

    if (msg.r2 !== undefined) {
      resolved.r2 = msg.r2;
    }
    else {
      resolved.r2 = []
    }

    if (msg.theta2 !== undefined) {
      resolved.theta2 = msg.theta2;
    }
    else {
      resolved.theta2 = []
    }

    if (msg.x1 !== undefined) {
      resolved.x1 = msg.x1;
    }
    else {
      resolved.x1 = []
    }

    if (msg.y1 !== undefined) {
      resolved.y1 = msg.y1;
    }
    else {
      resolved.y1 = []
    }

    if (msg.x2 !== undefined) {
      resolved.x2 = msg.x2;
    }
    else {
      resolved.x2 = []
    }

    if (msg.y2 !== undefined) {
      resolved.y2 = msg.y2;
    }
    else {
      resolved.y2 = []
    }

    if (msg.euc_length !== undefined) {
      resolved.euc_length = msg.euc_length;
    }
    else {
      resolved.euc_length = []
    }

    if (msg.delta_angle !== undefined) {
      resolved.delta_angle = msg.delta_angle;
    }
    else {
      resolved.delta_angle = []
    }

    return resolved;
    }
};

module.exports = gaps;
