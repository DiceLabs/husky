#!/usr/bin/env python3

from client import call_service
from services import ServiceNames, ServicePorts
from defaults import Defaults

print(call_service(port=ServicePorts[ServiceNames.RIGHT_CAMERA], request=Defaults.Trigger))
print(call_service(port=ServicePorts[ServiceNames.LEFT_CAMERA], request=Defaults.Trigger))