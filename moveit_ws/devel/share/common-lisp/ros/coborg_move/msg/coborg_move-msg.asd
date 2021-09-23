
(cl:in-package :asdf)

(defsystem "coborg_move-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CartesianTrajectory" :depends-on ("_package_CartesianTrajectory"))
    (:file "_package_CartesianTrajectory" :depends-on ("_package"))
  ))