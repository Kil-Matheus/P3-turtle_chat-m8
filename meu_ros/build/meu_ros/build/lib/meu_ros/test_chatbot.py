#! /usr/bin/env python3 
import rclpy
import re
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

rclpy.init()
nav = BasicNavigator()

my_dict = {
    'fim de turno' : 'sair',
    'sair' : "sair",
    'finalizar' : "sair",
    'inicio' : "0.0, 0.0, 0.0",
    'chave de fenda' : "1.03, 0.0, 0.0",
    'puro malte' : "3.61, 0.04, 0.0",
    'correia' : "2.56, 2.34, 0.0",
    'embalagem' : "1.74, 1.08, 0.0",
    'ponto 1' : "0.33, 2.0, 0.0",
}

def translate(text):
    nav = BasicNavigator()
    point = re.compile(r'\b(?:ponto\s?(\d+?)|chave\s?de\s?fenda|puro\s?malte|correia|embalagem)\b', re.IGNORECASE)
    zero_pose = re.compile(r'\b(?:inicio|inicial)\b', re.IGNORECASE)
    sair = re.compile(r'\b(?:sair|\s?finalizar|fim\s?de\s?turno)\b', re.IGNORECASE)
    match_sair = sair.search(text)
    match_point = point.search(text)
    match_zero_pose = zero_pose.search(text)
   
    if match_point:
        cood_point = my_dict.get(match_point.group(0), 'Coordenada inextistente nesse ponto') # Por causa da função get, ele pega o valor da chave.
        resp_p = f'Indo para o {match_point.group(0)}: {cood_point}'
        print(resp_p)
        arry_coordenadas = cood_point.split(',')
        x = float(arry_coordenadas[0])
        y = float(arry_coordenadas[1])
        z = float(arry_coordenadas[2])
        return create_pose_stamped(nav, x, y, z)
    
    elif match_zero_pose:
        cood_point = my_dict.get(match_zero_pose.group(0), '')
        resp_z = f'Retornando o {match_zero_pose.group(0)}: {cood_point}'
        print(resp_z)
        return create_pose_stamped(nav, 0.0, 0.0, 0.0)
    
    elif match_sair:
        print('Finalizando')
        return str(my_dict.get(match_sair.group(0)))
    
    return 0


def create_pose_stamped(navigator, pos_x, pos_y, rot_z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rot_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = pos_x
    pose.pose.position.y = pos_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    while True:
        waypoints = []
        text_input = input('Digite o ponto que deseja ir: ')
        waypoint = translate(text_input)

        if waypoint == 'sair':
            break

        if waypoint == 0:
            print('Coordenada inexistente')
            continue

        waypoints.append(waypoint)

        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            print(nav.getFeedback())

    rclpy.shutdown()


if __name__ == '__main__':
    main()