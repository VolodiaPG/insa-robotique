; Auto-generated. Do not edit!


(cl:in-package ropigo-srv)


;//! \htmlinclude SimpleWrite-request.msg.html

(cl:defclass <SimpleWrite-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SimpleWrite-request (<SimpleWrite-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimpleWrite-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimpleWrite-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ropigo-srv:<SimpleWrite-request> is deprecated: use ropigo-srv:SimpleWrite-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimpleWrite-request>) ostream)
  "Serializes a message object of type '<SimpleWrite-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimpleWrite-request>) istream)
  "Deserializes a message object of type '<SimpleWrite-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimpleWrite-request>)))
  "Returns string type for a service object of type '<SimpleWrite-request>"
  "ropigo/SimpleWriteRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimpleWrite-request)))
  "Returns string type for a service object of type 'SimpleWrite-request"
  "ropigo/SimpleWriteRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimpleWrite-request>)))
  "Returns md5sum for a message object of type '<SimpleWrite-request>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimpleWrite-request)))
  "Returns md5sum for a message object of type 'SimpleWrite-request"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimpleWrite-request>)))
  "Returns full string definition for message of type '<SimpleWrite-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimpleWrite-request)))
  "Returns full string definition for message of type 'SimpleWrite-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimpleWrite-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimpleWrite-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SimpleWrite-request
))
;//! \htmlinclude SimpleWrite-response.msg.html

(cl:defclass <SimpleWrite-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SimpleWrite-response (<SimpleWrite-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimpleWrite-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimpleWrite-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ropigo-srv:<SimpleWrite-response> is deprecated: use ropigo-srv:SimpleWrite-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SimpleWrite-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ropigo-srv:status-val is deprecated.  Use ropigo-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimpleWrite-response>) ostream)
  "Serializes a message object of type '<SimpleWrite-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimpleWrite-response>) istream)
  "Deserializes a message object of type '<SimpleWrite-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimpleWrite-response>)))
  "Returns string type for a service object of type '<SimpleWrite-response>"
  "ropigo/SimpleWriteResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimpleWrite-response)))
  "Returns string type for a service object of type 'SimpleWrite-response"
  "ropigo/SimpleWriteResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimpleWrite-response>)))
  "Returns md5sum for a message object of type '<SimpleWrite-response>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimpleWrite-response)))
  "Returns md5sum for a message object of type 'SimpleWrite-response"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimpleWrite-response>)))
  "Returns full string definition for message of type '<SimpleWrite-response>"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimpleWrite-response)))
  "Returns full string definition for message of type 'SimpleWrite-response"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimpleWrite-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimpleWrite-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SimpleWrite-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SimpleWrite)))
  'SimpleWrite-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SimpleWrite)))
  'SimpleWrite-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimpleWrite)))
  "Returns string type for a service object of type '<SimpleWrite>"
  "ropigo/SimpleWrite")