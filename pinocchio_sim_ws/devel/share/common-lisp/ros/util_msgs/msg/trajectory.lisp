; Auto-generated. Do not edit!


(cl:in-package util_msgs-msg)


;//! \htmlinclude trajectory.msg.html

(cl:defclass <trajectory> (roslisp-msg-protocol:ros-message)
  ((force_left
    :reader force_left
    :initarg :force_left
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (force_right
    :reader force_right
    :initarg :force_right
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (dp_left
    :reader dp_left
    :initarg :dp_left
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (dp_right
    :reader dp_right
    :initarg :dp_right
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass trajectory (<trajectory>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <trajectory>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'trajectory)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name util_msgs-msg:<trajectory> is deprecated: use util_msgs-msg:trajectory instead.")))

(cl:ensure-generic-function 'force_left-val :lambda-list '(m))
(cl:defmethod force_left-val ((m <trajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader util_msgs-msg:force_left-val is deprecated.  Use util_msgs-msg:force_left instead.")
  (force_left m))

(cl:ensure-generic-function 'force_right-val :lambda-list '(m))
(cl:defmethod force_right-val ((m <trajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader util_msgs-msg:force_right-val is deprecated.  Use util_msgs-msg:force_right instead.")
  (force_right m))

(cl:ensure-generic-function 'dp_left-val :lambda-list '(m))
(cl:defmethod dp_left-val ((m <trajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader util_msgs-msg:dp_left-val is deprecated.  Use util_msgs-msg:dp_left instead.")
  (dp_left m))

(cl:ensure-generic-function 'dp_right-val :lambda-list '(m))
(cl:defmethod dp_right-val ((m <trajectory>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader util_msgs-msg:dp_right-val is deprecated.  Use util_msgs-msg:dp_right instead.")
  (dp_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <trajectory>) ostream)
  "Serializes a message object of type '<trajectory>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'force_left) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'force_right) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dp_left) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dp_right) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <trajectory>) istream)
  "Deserializes a message object of type '<trajectory>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'force_left) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'force_right) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dp_left) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dp_right) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<trajectory>)))
  "Returns string type for a message object of type '<trajectory>"
  "util_msgs/trajectory")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'trajectory)))
  "Returns string type for a message object of type 'trajectory"
  "util_msgs/trajectory")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<trajectory>)))
  "Returns md5sum for a message object of type '<trajectory>"
  "5b6b28c558698fc8c155dd7d02d7a674")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'trajectory)))
  "Returns md5sum for a message object of type 'trajectory"
  "5b6b28c558698fc8c155dd7d02d7a674")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<trajectory>)))
  "Returns full string definition for message of type '<trajectory>"
  (cl:format cl:nil "geometry_msgs/Vector3 force_left~%geometry_msgs/Vector3 force_right~%geometry_msgs/Vector3 dp_left~%geometry_msgs/Vector3 dp_right~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'trajectory)))
  "Returns full string definition for message of type 'trajectory"
  (cl:format cl:nil "geometry_msgs/Vector3 force_left~%geometry_msgs/Vector3 force_right~%geometry_msgs/Vector3 dp_left~%geometry_msgs/Vector3 dp_right~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <trajectory>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'force_left))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'force_right))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dp_left))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dp_right))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <trajectory>))
  "Converts a ROS message object to a list"
  (cl:list 'trajectory
    (cl:cons ':force_left (force_left msg))
    (cl:cons ':force_right (force_right msg))
    (cl:cons ':dp_left (dp_left msg))
    (cl:cons ':dp_right (dp_right msg))
))
