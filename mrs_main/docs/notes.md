# Przydzielanie zadań w systemie wielorobotwym 

## Planowanie ścieżki 

Przykładowa komenda pozwalające na uzyskanie planu:

`rosservice call /robot1/move_base_node/NavfnROS/make_plan`

Wygodne korzystanie z serwisów zapewnia:

`rosrun rqt_service_caller rqt_service_caller`

Poprawna konfiguracja wywołania serwisu:

![proper_make_plan](photos/proper_make_plan/proper_make_plan_call.png)


Przykład wywołania serwisu w kodzie:
<pre><code>
make_plan = rospy.ServiceProxy('/robot1/move_base_node/NavfnROS/make_plan', GetPlan)
req = GetPlanRequest()

req.start.header.frame_id = 'map'
req.start.pose.position.x = 1.34
req.start.pose.position.x = 0.37
req.start.pose.orientation.w = 1

req.goal.header.frame_id = 'map'
req.goal.pose.position.x = -4
req.goal.pose.position.x = 1
req.goal.pose.orientation.w = 1

req.tolerance = 0.1
res = make_plan(req)

</code></pre>

Wygodna konfiguracja parametrów plannera:

`rosrun rqt_reconfigure rqt_reconfigure`



