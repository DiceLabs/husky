#!/usr/bin/env python3

from multiprocessing import Process
from services import ServiceNames, ServicePorts
from server import start_server
from base import BaseNode
from req_resp import GenericRequest

def generic_callback(request: GenericRequest, base: BaseNode):
    function_call = getattr(base, request.function)
    function_call(**request.args)

def start_base_server(name, port):
    base = BaseNode()
    start_server(name=name, port=port, callback=lambda request: generic_callback(request, base))

def start_base(service_name, service_port):
    process = Process(target=start_base_server, args=(service_name, service_port))
    process.start()
    process.join()

if __name__ == "__main__":
    start_base(ServiceNames.BASE,  ServicePorts[ServiceNames.BASE])
