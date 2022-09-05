// Auto-generated. Do not edit!

// (in-package action_demo.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ActionDemoFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.progress_bar = null;
    }
    else {
      if (initObj.hasOwnProperty('progress_bar')) {
        this.progress_bar = initObj.progress_bar
      }
      else {
        this.progress_bar = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActionDemoFeedback
    // Serialize message field [progress_bar]
    bufferOffset = _serializer.float64(obj.progress_bar, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActionDemoFeedback
    let len;
    let data = new ActionDemoFeedback(null);
    // Deserialize message field [progress_bar]
    data.progress_bar = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'action_demo/ActionDemoFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac7532a8a8f0d2be4f5284d94fee2d59';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    #连续反馈
    float64 progress_bar
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActionDemoFeedback(null);
    if (msg.progress_bar !== undefined) {
      resolved.progress_bar = msg.progress_bar;
    }
    else {
      resolved.progress_bar = 0.0
    }

    return resolved;
    }
};

module.exports = ActionDemoFeedback;
