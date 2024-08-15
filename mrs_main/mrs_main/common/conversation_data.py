from enum import Enum


class MrsConvPerform(str, Enum):
    propose_exec_role = 'propose_exec_role'
    accept_exec_proposal = 'acc_exec_proposal'
    declare_ex_intrest = 'dec_ex_intr'
    declare_coord_intrest = 'dec_coord_intr'
    # TODO: Decide if below should be removed
    propose_coord_role = 'propose_coord_role' 
    accept_coord_proposal = 'acc_coord_proposal'
