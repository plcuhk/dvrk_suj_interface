# Import the Client from ambf_comm package
# You might have to do: pip install gym
from ambf_client import Client
import time

import dvrk
from pprint import pprint

# dvrk setup
suj_psm1 = dvrk.suj('PSM1')
suj_psm2 = dvrk.suj('PSM2')
suj_ecm = dvrk.suj('ECM')


def print_all_status_once():
    print('PSM1 joints status')
    pprint(suj_psm1.measured_js())

    print('PSM2 joints status')
    pprint(suj_psm2.measured_js())

    print('ECM joints status')
    pprint(suj_ecm.measured_js())


# ambf setup
suj_ecm_joint_names = ['baselink-sujecmL0', 'sujecmL0-sujecmL1',
                       'sujecmL1-sujecmL2', 'sujecmL2-sujecmL3']
suj_psm1_joint_names = ['baselink-sujpsm1L0', 'sujpsm1L0-sujpsm1L1',
                        'sujpsm1L1-sujpsm1L2', 'sujpsm1L2-sujpsm1L3', 'sujpsm1L3-sujpsm1L4']
suj_psm2_joint_names = ['baselink-sujpsm2L0', 'sujpsm2L0-sujpsm2L1',
                        'sujpsm2L1-sujpsm2L2', 'sujpsm2L2-sujpsm2L3', 'sujpsm2L3-sujpsm2L4']


def update_suj_joints_pos():
    for zipped in [zip(suj_ecm_joint_names, suj_ecm.measured_js()[0]),
                   zip(suj_psm1_joint_names, suj_psm1.measured_js()[0]),
                   zip(suj_psm2_joint_names, suj_psm2.measured_js()[0])]:
        for name, pos in zipped:
            base_link.set_joint_pos(name, pos)


if __name__ == '__main__':
    print_all_status_once()

    # Create a instance of the client
    _client = Client()

    # Connect the client which in turn creates callable objects from ROS topics
    # and initiates a shared pool of threads for bi-directional communication
    _client.connect()

    print(_client.get_obj_names())

    base_link = _client.get_obj_handle('/ambf/env/baselink')
    joint_names = base_link.get_joint_names()

    while True:
        update_suj_joints_pos()
        time.sleep(0.01)
