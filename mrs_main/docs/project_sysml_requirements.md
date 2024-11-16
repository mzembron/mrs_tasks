### Requirements the mrs_tasks project should aim to fulfill
(to be moved to SysML requirements diagram)


#### Requirements:
- task allocation to a robot able to execute the task or explicit information of task rejection - **Explicit information on task acceptance**,
- all robots should be aligned with the plan of task execution (and allocation of tasks), actions should not colide - **System agents act according to the same plan**,
- ability to maintain the operations on the failure of any node (not all at once) - **Node failure proof (Availability)** ,
- optimization of execution time (with proper allocation), to be more specific: the output plan should not take more than approach with all tasks scheduled for one universal robot - **Task execution optimization**,
- provide updates on advancements in task execution - **Constant and regular updates (liveness)**,
- schedule actions (tasks) of specific robot (plan its future actions) - **Scheduling/Planning of future actions**,
- possibility to include into a system robot of different natures (heterogenic) - **Management of heterogenous robots**,
- ability to maintain reliability/operability with an increasing number of robots/agents in the system (scalable solution) - **Scalability**.