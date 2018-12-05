Header header
float64 joint_value 		#joint angle (to reduce number of degrees)
float64 JacobianDeterminant 	#manipulability criteria to avoid singularities
bool collision_free 		#determines if pose is viable (collision-free and solveable by IK)
bool isPoint			#determines if value is part of trajectory
