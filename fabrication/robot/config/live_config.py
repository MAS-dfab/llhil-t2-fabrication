import requests
from requests.auth import HTTPDigestAuth

def get_robot_data(robot_ip="192.168.0.11", username="Default User", password="robotics"):
    url_gantry = f"http://{robot_ip}/rw/motionsystem/mechunits/Gantry12/jointtarget?json=1"
    url_rob = f"http://{robot_ip}/rw/motionsystem/mechunits/ROB_2/jointtarget?json=1"
    
    try:
        response_gantry = requests.get(url_gantry, auth=HTTPDigestAuth(username, password), timeout=5)
        response_rob = requests.get(url_rob, auth=HTTPDigestAuth(username, password), timeout=5)
        
        if response_gantry.status_code == 200 and response_rob.status_code == 200:
            
            g_state = response_gantry.json().get('_embedded', {}).get('_state', [{}])[0]
            r_state = response_rob.json().get('_embedded', {}).get('_state', [{}])[0]
            
            gantry_x = float(g_state.get('rax_1', 0))
            gantry_y = float(g_state.get('rax_2', 0))
            gantry_z = float(g_state.get('rax_3', 0))
            
            rob_j1 = float(r_state.get('rax_1', 0))
            rob_j2 = float(r_state.get('rax_2', 0))
            rob_j3 = float(r_state.get('rax_3', 0))
            rob_j4 = float(r_state.get('rax_4', 0))
            rob_j5 = float(r_state.get('rax_5', 0))
            rob_j6 = float(r_state.get('rax_6', 0))

            print(gantry_x, gantry_y, gantry_z, rob_j1, rob_j2, rob_j3, rob_j4, rob_j5, rob_j6)
            
            return [gantry_x, gantry_y, gantry_z, rob_j1, rob_j2, rob_j3, rob_j4, rob_j5, rob_j6]
            
        elif response_gantry.status_code == 401 or response_rob.status_code == 401:
            print("Error 401: Unauthorized. Check your USERNAME and PASSWORD.")
            return None
        else:
            print(f"Error: Gantry returned {response_gantry.status_code}, Robot returned {response_rob.status_code}.")
            return None
            
    except requests.exceptions.RequestException as e:
        print(f"Connection failed: {e}")
        return None