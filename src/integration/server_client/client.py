#!/usr/bin/env python3

import socket
from defaults import Defaults
import pickle

def start_client(host=Defaults.LOCALHOST, port=Defaults.PORT, request=Defaults.request):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        raw_request = pickle.dumps(request)
        s.sendall(raw_request)
        raw_response = s.recv(Defaults.BUFFER_SIZE)
        if raw_response:
            response = pickle.loads(raw_response)
            print(response)
    return response

if __name__ == "__main__":
    start_client()