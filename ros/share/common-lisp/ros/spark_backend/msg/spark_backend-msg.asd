
(cl:in-package :asdf)

(defsystem "spark_backend-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AuboInfo" :depends-on ("_package_AuboInfo"))
    (:file "_package_AuboInfo" :depends-on ("_package"))
    (:file "OmronPLCInfo" :depends-on ("_package_OmronPLCInfo"))
    (:file "_package_OmronPLCInfo" :depends-on ("_package"))
    (:file "RobotServiceAction" :depends-on ("_package_RobotServiceAction"))
    (:file "_package_RobotServiceAction" :depends-on ("_package"))
    (:file "RobotServiceActionFeedback" :depends-on ("_package_RobotServiceActionFeedback"))
    (:file "_package_RobotServiceActionFeedback" :depends-on ("_package"))
    (:file "RobotServiceActionGoal" :depends-on ("_package_RobotServiceActionGoal"))
    (:file "_package_RobotServiceActionGoal" :depends-on ("_package"))
    (:file "RobotServiceActionResult" :depends-on ("_package_RobotServiceActionResult"))
    (:file "_package_RobotServiceActionResult" :depends-on ("_package"))
    (:file "RobotServiceFeedback" :depends-on ("_package_RobotServiceFeedback"))
    (:file "_package_RobotServiceFeedback" :depends-on ("_package"))
    (:file "RobotServiceGoal" :depends-on ("_package_RobotServiceGoal"))
    (:file "_package_RobotServiceGoal" :depends-on ("_package"))
    (:file "RobotServiceResult" :depends-on ("_package_RobotServiceResult"))
    (:file "_package_RobotServiceResult" :depends-on ("_package"))
    (:file "ServiceManagerAction" :depends-on ("_package_ServiceManagerAction"))
    (:file "_package_ServiceManagerAction" :depends-on ("_package"))
    (:file "ServiceManagerActionFeedback" :depends-on ("_package_ServiceManagerActionFeedback"))
    (:file "_package_ServiceManagerActionFeedback" :depends-on ("_package"))
    (:file "ServiceManagerActionGoal" :depends-on ("_package_ServiceManagerActionGoal"))
    (:file "_package_ServiceManagerActionGoal" :depends-on ("_package"))
    (:file "ServiceManagerActionResult" :depends-on ("_package_ServiceManagerActionResult"))
    (:file "_package_ServiceManagerActionResult" :depends-on ("_package"))
    (:file "ServiceManagerFeedback" :depends-on ("_package_ServiceManagerFeedback"))
    (:file "_package_ServiceManagerFeedback" :depends-on ("_package"))
    (:file "ServiceManagerGoal" :depends-on ("_package_ServiceManagerGoal"))
    (:file "_package_ServiceManagerGoal" :depends-on ("_package"))
    (:file "ServiceManagerResult" :depends-on ("_package_ServiceManagerResult"))
    (:file "_package_ServiceManagerResult" :depends-on ("_package"))
    (:file "VisionServiceAction" :depends-on ("_package_VisionServiceAction"))
    (:file "_package_VisionServiceAction" :depends-on ("_package"))
    (:file "VisionServiceActionFeedback" :depends-on ("_package_VisionServiceActionFeedback"))
    (:file "_package_VisionServiceActionFeedback" :depends-on ("_package"))
    (:file "VisionServiceActionGoal" :depends-on ("_package_VisionServiceActionGoal"))
    (:file "_package_VisionServiceActionGoal" :depends-on ("_package"))
    (:file "VisionServiceActionResult" :depends-on ("_package_VisionServiceActionResult"))
    (:file "_package_VisionServiceActionResult" :depends-on ("_package"))
    (:file "VisionServiceFeedback" :depends-on ("_package_VisionServiceFeedback"))
    (:file "_package_VisionServiceFeedback" :depends-on ("_package"))
    (:file "VisionServiceGoal" :depends-on ("_package_VisionServiceGoal"))
    (:file "_package_VisionServiceGoal" :depends-on ("_package"))
    (:file "VisionServiceResult" :depends-on ("_package_VisionServiceResult"))
    (:file "_package_VisionServiceResult" :depends-on ("_package"))
  ))