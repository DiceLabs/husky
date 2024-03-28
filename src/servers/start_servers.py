import subprocess

BASE_SERVICE =    ["base_service.py"]
CAMERA_SERVICE =  ["camera_service.py"]
GRIPPER_SERVICE = ["gripper_service.py"]

if __name__ == "__main__":
    scripts = [CAMERA_SERVICE, GRIPPER_SERVICE, BASE_SERVICE]
    for script in scripts:
            process = subprocess.Popen(script)