import subprocess

BASE_CLIENT =    ["base_client.py"]
CAMERA_CLIENT =  ["camera_client.py"]
GRIPPER_CLIENT = ["gripper_client.py"]

if __name__ == "__main__":
    scripts = [CAMERA_CLIENT, GRIPPER_CLIENT, BASE_CLIENT]
    for script in scripts:
            process = subprocess.Popen(script)