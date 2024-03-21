""" 
    @Zix 
    Generic ROS2 Client API to help make function calls
    Basically to integrate with any service we write, you would have to figure out how to properly make
    the service call and also pass the appropriate parameters. Instead, each service can be shipped with 
    the appropriate client code so someone can use one public function to integrate with another package.
    Every package that needs to interface with another module could import this central module
"""

import rospy

###################################################################################################################


""" 
    GENERIC ROS2 SERVICE CALL CODE  
    This will wrap validation and temp node spinning to process callback.
    This always has to do be done no matter the service call, so we can write it once and leave boilerplate out of main code
"""


def validate_service(client) -> bool:
        while not client.wait_for_service(10):
            if not rospy.ok():
                return False
        return True


def make_service_call(node, client, request): 
    """ 
        Each call will need a temp @param node with a @param client attached.
        Since we have type inference in python we can pass in any request
        @return The response value of the particular service
    """
    future = client.call_async(request)
    if not validate_service(client):
        return None
    rospy.spin_until_future_complete(node, future, timeout_sec=0.1)
    return future.result()


################################################################################################################### 