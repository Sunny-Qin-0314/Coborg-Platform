
(cl:in-package :asdf)

(defsystem "hebi_cpp_api_examples-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetCommandLifetime" :depends-on ("_package_SetCommandLifetime"))
    (:file "_package_SetCommandLifetime" :depends-on ("_package"))
    (:file "SetFeedbackFrequency" :depends-on ("_package_SetFeedbackFrequency"))
    (:file "_package_SetFeedbackFrequency" :depends-on ("_package"))
    (:file "SetGains" :depends-on ("_package_SetGains"))
    (:file "_package_SetGains" :depends-on ("_package"))
    (:file "SetIKSeed" :depends-on ("_package_SetIKSeed"))
    (:file "_package_SetIKSeed" :depends-on ("_package"))
  ))