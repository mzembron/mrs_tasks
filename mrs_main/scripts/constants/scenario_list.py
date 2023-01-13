###
# List of defined scenarios
# Main purpose of this file is to indicate whether file is scenario or not
###

# prev version
# SCENARIO_LIST = {  
#                     'BG' : {    #BRING GOOD SCENARIO
#                             "subtasks" : ['MT', 'GT', 'MT'],
#                             "data distribution" : [0, 0, 1 ]
#                         } 
#                 }
        
SCENARIO_LIST = {
                    'BG': {    # BRING GOOD SCENARIO
                            "subtasks": [
                                {
                                        "type": 'GT',
                                        "index": 0,
                                        "appropriate data index": 0,  
                                        # refers to data list passed as Task
                                        #  class atribute
                                        "requires to start": [],
                                        "requires to end": [],
                                        "same robot as task": None}, 
                                {
                                        "type": 'MT',
                                        "index": 1,
                                        "appropriate data index": 0,
                                        "requires to start": [],
                                        "requires to end": [0],
                                        "same robot as task": None},
                                {
                                        "type": 'GT',
                                        "index": 2,
                                        "appropriate data index": 1,
                                        "requires to start": [1],
                                        "requires to end": [],
                                        "same robot as task": 0}]
                        },

                        'DBG1': {    # Debug 1 scenario - start condition
                            "subtasks": [
                                {
                                        "type": 'GT',
                                        "index": 0,
                                        "appropriate data index": 0,  
                                        # refers to data list passed as Task
                                        #  class atribute
                                        "requires to start": [],
                                        "requires to end": [],
                                        "same robot as task": None}, 
                                {
                                        "type": 'MT',
                                        "index": 1,
                                        "appropriate data index": 1,
                                        "requires to start": [0],
                                        "requires to end": [],
                                        "same robot as task": None}]
                        },
                        'DBG2': {    # Debug 2 scenario - end condition
                            "subtasks": [
                                {
                                        "type": 'GT',
                                        "index": 0,
                                        "appropriate data index": 0,  
                                        # refers to data list passed as Task
                                        #  class atribute
                                        "requires to start": [],
                                        "requires to end": [],
                                        "same robot as task": None}, 
                                {
                                        "type": 'MT',
                                        "index": 1,
                                        "appropriate data index": 1,
                                        "requires to start": [],
                                        "requires to end": [0],
                                        "same robot as task": None}]
                        },
                        'DBG3': {    # Debug 2 scenario - similar to BG
                            "subtasks": [
                                {
                                        "type": 'GT',
                                        "index": 0,
                                        "appropriate data index": 0,  
                                        # refers to data list passed as Task
                                        #  class atribute
                                        "requires to start": [],
                                        "requires to end": [],
                                        "same robot as task": None}, 
                                {
                                        "type": 'MT',
                                        "index": 1,
                                        "appropriate data index": 1,
                                        "requires to start": [],
                                        "requires to end": [0], # debug 0 deleted
                                        "same robot as task": None},
                                {
                                        "type": 'GT',
                                        "index": 2,
                                        "appropriate data index": 2,
                                        "requires to start": [1],
                                        "requires to end": [],
                                        "same robot as task": 0}]
                        },
                        'DBG4': {    
                            "subtasks": [
                                {
                                        "type": 'GT',
                                        "index": 0,
                                        "appropriate data index": 0,  
                                        "requires to start": [],
                                        "requires to end": [],
                                        "same robot as task": None}, 
                                {
                                        "type": 'GT',
                                        "index": 1,
                                        "appropriate data index": 1,
                                        "requires to start": [0],
                                        "requires to end": [],
                                        "same robot as task": None}]
                        },
                        'DBG5': {    
                            "subtasks": [
                                {
                                        "type": 'GT',
                                        "index": 0,
                                        "appropriate data index": 0,  
                                        # refers to data list passed as Task
                                        #  class atribute
                                        "requires to start": [],
                                        "requires to end": [],
                                        "same robot as task": None}, 
                                {
                                        "type": 'GT',
                                        "index": 1,
                                        "appropriate data index": 1,
                                        "requires to start": [],
                                        "requires to end": [0], 
                                        "same robot as task": None},
                                {
                                        "type": 'GT',
                                        "index": 2,
                                        "appropriate data index": 2,
                                        "requires to start": [0, 1],
                                        "requires to end": [],
                                        "same robot as task": 0}]
                        }



                }
