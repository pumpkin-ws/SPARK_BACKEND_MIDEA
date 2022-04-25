
(cl:in-package :asdf)

(defsystem "spark_backend-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RobotClientServer" :depends-on ("_package_RobotClientServer"))
    (:file "_package_RobotClientServer" :depends-on ("_package"))
    (:file "VisionClientServer" :depends-on ("_package_VisionClientServer"))
    (:file "_package_VisionClientServer" :depends-on ("_package"))
  ))