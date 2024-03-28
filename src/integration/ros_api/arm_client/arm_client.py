#!/usr/bin/env python3

from client import call_service
from services import ServiceNames, ServicePorts
from req_resp import GenericRequest

call_service(port=ServicePorts[ServiceNames.LEFT_ARM], request=GenericRequest(function="move_up", args={"blocking":False}))
call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(function="move_up", args={"blocking":True}))
call_service(port=ServicePorts[ServiceNames.LEFT_ARM], request=GenericRequest(function="move_down", args={"blocking":False}))
call_service(port=ServicePorts[ServiceNames.RIGHT_ARM], request=GenericRequest(function="move_down", args={"blocking":True}))