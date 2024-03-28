#!/usr/bin/env python3

from client import call_service
from services import ServiceNames, ServicePorts
from defaults import Defaults

response = call_service(port=ServicePorts[ServiceNames.RIGHT_CAMERA], request=Defaults.Trigger)
if response:
    print(response)