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
                    'BG' : {    #BRING GOOD SCENARIO
                            "subtasks" : [
                                {
                                        "type": 'MT',
                                        "appropriate data index": 0 }, 
                                {       
                                        "type": 'GT',
                                        "appropriate data index": 0 },      
                                {        
                                        "type": 'MT',
                                        "appropriate data index": 1 }] 
                        } 
                }
