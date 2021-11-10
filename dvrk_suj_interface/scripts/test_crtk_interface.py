import dvrk
from pprint import pprint

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


if __name__ == '__main__':
    print_all_status_once()
