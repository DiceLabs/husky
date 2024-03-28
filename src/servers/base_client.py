#!/usr/bin/env python3

from client import call_service
from services import ServiceNames, ServicePorts
from defaults import Defaults

call_service(port=ServicePorts[ServiceNames.BASE], request=Defaults.Trigger)
