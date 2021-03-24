
(cl:in-package :asdf)

(defsystem "ros_detect_planes_from_depth_img-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PlanesResults" :depends-on ("_package_PlanesResults"))
    (:file "_package_PlanesResults" :depends-on ("_package"))
  ))