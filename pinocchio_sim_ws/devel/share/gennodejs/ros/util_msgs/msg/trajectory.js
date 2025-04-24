// Auto-generated. Do not edit!

// (in-package util_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class trajectory {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.force_left = null;
      this.force_right = null;
      this.dp_left = null;
      this.dp_right = null;
    }
    else {
      if (initObj.hasOwnProperty('force_left')) {
        this.force_left = initObj.force_left
      }
      else {
        this.force_left = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('force_right')) {
        this.force_right = initObj.force_right
      }
      else {
        this.force_right = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('dp_left')) {
        this.dp_left = initObj.dp_left
      }
      else {
        this.dp_left = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('dp_right')) {
        this.dp_right = initObj.dp_right
      }
      else {
        this.dp_right = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type trajectory
    // Serialize message field [force_left]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.force_left, buffer, bufferOffset);
    // Serialize message field [force_right]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.force_right, buffer, bufferOffset);
    // Serialize message field [dp_left]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.dp_left, buffer, bufferOffset);
    // Serialize message field [dp_right]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.dp_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type trajectory
    let len;
    let data = new trajectory(null);
    // Deserialize message field [force_left]
    data.force_left = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [force_right]
    data.force_right = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [dp_left]
    data.dp_left = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [dp_right]
    data.dp_right = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'util_msgs/trajectory';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5b6b28c558698fc8c155dd7d02d7a674';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Vector3 force_left
    geometry_msgs/Vector3 force_right
    geometry_msgs/Vector3 dp_left
    geometry_msgs/Vector3 dp_right
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new trajectory(null);
    if (msg.force_left !== undefined) {
      resolved.force_left = geometry_msgs.msg.Vector3.Resolve(msg.force_left)
    }
    else {
      resolved.force_left = new geometry_msgs.msg.Vector3()
    }

    if (msg.force_right !== undefined) {
      resolved.force_right = geometry_msgs.msg.Vector3.Resolve(msg.force_right)
    }
    else {
      resolved.force_right = new geometry_msgs.msg.Vector3()
    }

    if (msg.dp_left !== undefined) {
      resolved.dp_left = geometry_msgs.msg.Vector3.Resolve(msg.dp_left)
    }
    else {
      resolved.dp_left = new geometry_msgs.msg.Vector3()
    }

    if (msg.dp_right !== undefined) {
      resolved.dp_right = geometry_msgs.msg.Vector3.Resolve(msg.dp_right)
    }
    else {
      resolved.dp_right = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = trajectory;
