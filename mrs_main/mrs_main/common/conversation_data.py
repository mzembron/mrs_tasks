from enum import Enum


class MrsConvPerform(str, Enum):
    propose_exec_role = 'propose_exec_role'
    accept_exec_proposal = 'acc_exec_proposal'
    declare_ex_intrest = 'dec_ex_intr'
    declare_coord_intrest = 'dec_coord_intr'
    request_exec_info = 'req_exec_info'

