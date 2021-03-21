; Auto-generated. Do not edit!


(cl:in-package ros_detect_planes_from_depth_img-msg)


;//! \htmlinclude PlanesResults.msg.html

(cl:defclass <PlanesResults> (roslisp-msg-protocol:ros-message)
  ((N
    :reader N
    :initarg :N
    :type cl:integer
    :initform 0)
   (norms
    :reader norms
    :initarg :norms
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (center_3d
    :reader center_3d
    :initarg :center_3d
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (center_2d
    :reader center_2d
    :initarg :center_2d
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (mask_color
    :reader mask_color
    :initarg :mask_color
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass PlanesResults (<PlanesResults>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanesResults>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanesResults)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_detect_planes_from_depth_img-msg:<PlanesResults> is deprecated: use ros_detect_planes_from_depth_img-msg:PlanesResults instead.")))

(cl:ensure-generic-function 'N-val :lambda-list '(m))
(cl:defmethod N-val ((m <PlanesResults>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_detect_planes_from_depth_img-msg:N-val is deprecated.  Use ros_detect_planes_from_depth_img-msg:N instead.")
  (N m))

(cl:ensure-generic-function 'norms-val :lambda-list '(m))
(cl:defmethod norms-val ((m <PlanesResults>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_detect_planes_from_depth_img-msg:norms-val is deprecated.  Use ros_detect_planes_from_depth_img-msg:norms instead.")
  (norms m))

(cl:ensure-generic-function 'center_3d-val :lambda-list '(m))
(cl:defmethod center_3d-val ((m <PlanesResults>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_detect_planes_from_depth_img-msg:center_3d-val is deprecated.  Use ros_detect_planes_from_depth_img-msg:center_3d instead.")
  (center_3d m))

(cl:ensure-generic-function 'center_2d-val :lambda-list '(m))
(cl:defmethod center_2d-val ((m <PlanesResults>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_detect_planes_from_depth_img-msg:center_2d-val is deprecated.  Use ros_detect_planes_from_depth_img-msg:center_2d instead.")
  (center_2d m))

(cl:ensure-generic-function 'mask_color-val :lambda-list '(m))
(cl:defmethod mask_color-val ((m <PlanesResults>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_detect_planes_from_depth_img-msg:mask_color-val is deprecated.  Use ros_detect_planes_from_depth_img-msg:mask_color instead.")
  (mask_color m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanesResults>) ostream)
  "Serializes a message object of type '<PlanesResults>"
  (cl:let* ((signed (cl:slot-value msg 'N)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'norms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'norms))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'center_3d))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'center_3d))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'center_2d))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'center_2d))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mask_color))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream))
   (cl:slot-value msg 'mask_color))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanesResults>) istream)
  "Deserializes a message object of type '<PlanesResults>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'N) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'norms) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'norms)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'center_3d) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'center_3d)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'center_2d) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'center_2d)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mask_color) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mask_color)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanesResults>)))
  "Returns string type for a message object of type '<PlanesResults>"
  "ros_detect_planes_from_depth_img/PlanesResults")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanesResults)))
  "Returns string type for a message object of type 'PlanesResults"
  "ros_detect_planes_from_depth_img/PlanesResults")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanesResults>)))
  "Returns md5sum for a message object of type '<PlanesResults>"
  "d1490bcb974cae216e975f12f5d851b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanesResults)))
  "Returns md5sum for a message object of type 'PlanesResults"
  "d1490bcb974cae216e975f12f5d851b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanesResults>)))
  "Returns full string definition for message of type '<PlanesResults>"
  (cl:format cl:nil "int32 N                 # Number of detected planes.~%~%# In the following arrays, ~%# the planes' parameters are concatinated one by one.~%~%float32[] norms         # Nx3. Plane normal (nx, ny, nz).~%float32[] center_3d     # Nx3. Plane 3D center (cx, cy, cz).~%float32[] center_2d     # Nx2. Plane 2D center on the image (px, py),~%                        #   which means {px}th column, and {py}th row.~%uint16[] mask_color     # Nx3. Plane mask color (blue, green, red). ~%                        # Each color's range is [0, 255]~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanesResults)))
  "Returns full string definition for message of type 'PlanesResults"
  (cl:format cl:nil "int32 N                 # Number of detected planes.~%~%# In the following arrays, ~%# the planes' parameters are concatinated one by one.~%~%float32[] norms         # Nx3. Plane normal (nx, ny, nz).~%float32[] center_3d     # Nx3. Plane 3D center (cx, cy, cz).~%float32[] center_2d     # Nx2. Plane 2D center on the image (px, py),~%                        #   which means {px}th column, and {py}th row.~%uint16[] mask_color     # Nx3. Plane mask color (blue, green, red). ~%                        # Each color's range is [0, 255]~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanesResults>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'norms) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'center_3d) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'center_2d) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mask_color) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanesResults>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanesResults
    (cl:cons ':N (N msg))
    (cl:cons ':norms (norms msg))
    (cl:cons ':center_3d (center_3d msg))
    (cl:cons ':center_2d (center_2d msg))
    (cl:cons ':mask_color (mask_color msg))
))
