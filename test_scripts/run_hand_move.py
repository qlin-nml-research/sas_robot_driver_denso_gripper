import rospy
from bcap_service.srv import bcap, bcapResponse, bcapRequest
from bcap_service.msg import variant
import time


def main():
    rospy.init_node('test_gripper', anonymous=True)
    service_h = rospy.ServiceProxy('bcap_service', bcap)

    # controller_id = get_controller_id(service_h)
    controller_id = "12"
    print("got controller", controller_id)
    # time.sleep(1)
    move_with(
        service_h,
        controller_id,
        5.6, 100
    )

def get_controller_id(bcap_service):
    # create handler to call the service

    # define the request
    func_id = 3
    vt = 8
    values = ['b-CAP', 'CaoProv.DENSO.VRC', 'localhost', '']
    vts = [8, 8, 8, 8]
    variants = [variant(vt=v, value=val) for v, val in zip(vts, values)]
    req = bcapRequest()
    req.func_id = func_id
    req.vntArgs = variants
    print(req)

    # call the service by passing the request to the handler
    response: bcapResponse = bcap_service(req)
    print(response)
    controller_id = response.vntRet.value
    return controller_id

def move_with(bcap_service, controller_id, width, speed):
    # create handler to call the service
    print("requesting move", width, speed)
    # define the request
    func_id = 17
    vts = [19, 8, 8195]
    values = [controller_id, 'HandMoveA', f'{width}, {int(speed)}']
    print(values)
    vnts = [variant(vt=v, value=val) for v, val in zip(vts, values)]
    req = bcapRequest()
    req.func_id = func_id
    req.vntArgs = vnts
    # call the service by passing the request to the handler
    response: bcapResponse = bcap_service(req)
    print(response)


if __name__ == "__main__":
    main()