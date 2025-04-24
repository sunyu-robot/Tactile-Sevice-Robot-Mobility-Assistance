;; Auto-generated. Do not edit!


(when (boundp 'util_msgs::trajectory)
  (if (not (find-package "UTIL_MSGS"))
    (make-package "UTIL_MSGS"))
  (shadow 'trajectory (find-package "UTIL_MSGS")))
(unless (find-package "UTIL_MSGS::TRAJECTORY")
  (make-package "UTIL_MSGS::TRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude trajectory.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass util_msgs::trajectory
  :super ros::object
  :slots (_force_left _force_right _dp_left _dp_right ))

(defmethod util_msgs::trajectory
  (:init
   (&key
    ((:force_left __force_left) (instance geometry_msgs::Vector3 :init))
    ((:force_right __force_right) (instance geometry_msgs::Vector3 :init))
    ((:dp_left __dp_left) (instance geometry_msgs::Vector3 :init))
    ((:dp_right __dp_right) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _force_left __force_left)
   (setq _force_right __force_right)
   (setq _dp_left __dp_left)
   (setq _dp_right __dp_right)
   self)
  (:force_left
   (&rest __force_left)
   (if (keywordp (car __force_left))
       (send* _force_left __force_left)
     (progn
       (if __force_left (setq _force_left (car __force_left)))
       _force_left)))
  (:force_right
   (&rest __force_right)
   (if (keywordp (car __force_right))
       (send* _force_right __force_right)
     (progn
       (if __force_right (setq _force_right (car __force_right)))
       _force_right)))
  (:dp_left
   (&rest __dp_left)
   (if (keywordp (car __dp_left))
       (send* _dp_left __dp_left)
     (progn
       (if __dp_left (setq _dp_left (car __dp_left)))
       _dp_left)))
  (:dp_right
   (&rest __dp_right)
   (if (keywordp (car __dp_right))
       (send* _dp_right __dp_right)
     (progn
       (if __dp_right (setq _dp_right (car __dp_right)))
       _dp_right)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _force_left
    (send _force_left :serialization-length)
    ;; geometry_msgs/Vector3 _force_right
    (send _force_right :serialization-length)
    ;; geometry_msgs/Vector3 _dp_left
    (send _dp_left :serialization-length)
    ;; geometry_msgs/Vector3 _dp_right
    (send _dp_right :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _force_left
       (send _force_left :serialize s)
     ;; geometry_msgs/Vector3 _force_right
       (send _force_right :serialize s)
     ;; geometry_msgs/Vector3 _dp_left
       (send _dp_left :serialize s)
     ;; geometry_msgs/Vector3 _dp_right
       (send _dp_right :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _force_left
     (send _force_left :deserialize buf ptr-) (incf ptr- (send _force_left :serialization-length))
   ;; geometry_msgs/Vector3 _force_right
     (send _force_right :deserialize buf ptr-) (incf ptr- (send _force_right :serialization-length))
   ;; geometry_msgs/Vector3 _dp_left
     (send _dp_left :deserialize buf ptr-) (incf ptr- (send _dp_left :serialization-length))
   ;; geometry_msgs/Vector3 _dp_right
     (send _dp_right :deserialize buf ptr-) (incf ptr- (send _dp_right :serialization-length))
   ;;
   self)
  )

(setf (get util_msgs::trajectory :md5sum-) "5b6b28c558698fc8c155dd7d02d7a674")
(setf (get util_msgs::trajectory :datatype-) "util_msgs/trajectory")
(setf (get util_msgs::trajectory :definition-)
      "geometry_msgs/Vector3 force_left
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
")



(provide :util_msgs/trajectory "5b6b28c558698fc8c155dd7d02d7a674")


