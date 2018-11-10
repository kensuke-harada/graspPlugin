#sample of graspPlan

import grasp

try:
	tc= grasp.TaskController.instance(None)
	tc.doGraspPlanning()
	v = grasp.PythonConverter.Vector3(0,0,0.1)



except Exception, inst:
	print "graspPlan python Stopped"
