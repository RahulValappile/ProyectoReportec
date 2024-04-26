#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D

class PoseFilter:
    def __init__(self):
        # Inicializar el nodo de ROS
        rospy.init_node('pose_filter', anonymous=True)
        
        # Definir el publicador
        self.pub = rospy.Publisher('simple_pose', Pose2D, queue_size=10)
        
        # Suscribirse al tópico /poseupdate
        self.sub = rospy.Subscriber("/poseupdate", PoseWithCovarianceStamped, self.callback)

    def callback(self, data):
        # Crear mensaje Pose2D
        pose2d = Pose2D()
        pose2d.x = data.pose.pose.position.x
        pose2d.y = data.pose.pose.position.y
        pose2d.theta = data.pose.pose.orientation.w
        
        # Publicar el mensaje
        self.pub.publish(pose2d)
        rospy.logdebug("Publicado pose simplificado: %s", pose2d)

if __name__ == '__main__':
    pf = PoseFilter()
    try:
        # Configurar el nivel de logging
        rospy.loginfo("Iniciando nodo pose_filter")
        
        # Mantener el script en ejecución
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo pose_filter detenido.")
