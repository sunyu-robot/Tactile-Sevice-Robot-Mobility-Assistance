/*
 * Copyright (c) 2023 Yu Sun <sunyuu@zju.edu.cn>
 * All rights reserved.
 */
#include<tactile_compliance/CriticalPoint.hpp>

void CriticalPoint::initsphere(sphere (&point_sphere)[35])
{
    for(int i = 0; i< 35;i++)
    {
        point_sphere[i].radius = 0.001;
        point_sphere[i].collision_check.clear();
    }
    point_sphere[0].radius = 0.10;
    point_sphere[1].radius = 0.115;
    point_sphere[6].radius = 0.075;
    point_sphere[4].radius = 0.09;
    point_sphere[3].radius = 0.09;
    point_sphere[5].radius = 0.09;
    point_sphere[2].radius = 0.12;
    point_sphere[9].radius = 0.07;
    point_sphere[8].radius = 0.08;
    point_sphere[7].radius = 0.08;
    point_sphere[10].radius = 0.07;
    point_sphere[11].radius = 0.065;
    point_sphere[13].radius = 0.045;
    point_sphere[12].radius = 0.05;
    point_sphere[14].radius = 0.10;
    point_sphere[15].radius = 0.115;
    point_sphere[20].radius = 0.075;
    point_sphere[19].radius = 0.09;
    point_sphere[18].radius = 0.09;
    point_sphere[17].radius = 0.09;
    point_sphere[16].radius = 0.12;
    point_sphere[23].radius = 0.07;
    point_sphere[22].radius = 0.08;
    point_sphere[21].radius = 0.08;
    point_sphere[24].radius = 0.07;
    point_sphere[25].radius = 0.065;
    point_sphere[27].radius = 0.045;
    point_sphere[26].radius = 0.05;
    point_sphere[28].radius = 0.2;
    point_sphere[29].radius = 0.2;
    point_sphere[30].radius = 0.2;
    point_sphere[31].radius = 0.2;
    point_sphere[32].radius = 0.42;
    point_sphere[33].radius = 0.36;
    point_sphere[34].radius = 0.36;

    for(int i = 2; i < 14; i++)
    {
        // right arm && base
        for(int j = 14; j < 35; j++){
            point_sphere[i].collision_check.push_back(j);
        }
        // self
        for(int k = 0; k < 14; k++){
            if(k - i != 1 && k - i != -1)
            {
                point_sphere[i].collision_check.push_back(k);
            }
        }
    }
    
    for(int i = 16; i < 28; i++)
    {
        // left arm && base
        for(int j = 0; j < 14; j++){
            point_sphere[i].collision_check.push_back(j);
        }
        for(int j = 28; j < 35; j++){
            point_sphere[i].collision_check.push_back(j);
        }
        // self
        for(int k = 14; k < 28; k++){
            if(k - i != 1 &&  k - i != -1)
            {
                point_sphere[i].collision_check.push_back(k);
            }
        }
    }
}

void CriticalPoint::updatedata(const Vector15d q_real, sphere (&point_sphere)[35])
{
    for(int i = 0; i < 14; i++)
    {
        point_sphere[i].position = critiacal_position(i,q_real(0),q_real(1),q_real(2),q_real(3),q_real(4),q_real(5),q_real(6),q_real(7));
        point_sphere[i].jacobian_matrix = getJacobiantouch_cpoint(i,q_real(2),q_real(3),q_real(4),q_real(5),q_real(6),q_real(7),q_real(8));
    }
    for(int i = 14; i < 35; i++)
    {
        point_sphere[i].position = critiacal_position(i,q_real(0),q_real(1),q_real(2),q_real(9),q_real(10),q_real(11),q_real(12),q_real(13));
        point_sphere[i].jacobian_matrix = getJacobiantouch_cpoint(i,q_real(2),q_real(9),q_real(10),q_real(11),q_real(12),q_real(13),q_real(14));
    }
}

void CriticalPoint::pointMarker(visualization_msgs::MarkerArray &marker_array, const sphere* point_sphere)
{
    for(size_t i = 0;i < 35;i++)
    {
        sphere_marker[i].id = 10+i;
        sphere_marker[i].type = visualization_msgs::Marker::SPHERE;
        sphere_marker[i].ns = "critical_point";
        
        sphere_marker[i].pose.orientation.x = 0.0;
        sphere_marker[i].pose.orientation.y = 0.0;
        sphere_marker[i].pose.orientation.z = 0.0;
        sphere_marker[i].pose.orientation.w = 0.0;

        sphere_marker[i].header.frame_id = "world";

        sphere_marker[i].pose.position.x = point_sphere[i].position[0];
        sphere_marker[i].pose.position.y = point_sphere[i].position[1];
        sphere_marker[i].pose.position.z = point_sphere[i].position[2];

        sphere_marker[i].scale.x = point_sphere[i].radius * 2;
        sphere_marker[i].scale.y = point_sphere[i].radius * 2;
        sphere_marker[i].scale.z = point_sphere[i].radius * 2;

        sphere_marker[i].color.a = 0.7;
        sphere_marker[i].color.g = 0.6;
        
        marker_array.markers.push_back(sphere_marker[i]);
    }   

}