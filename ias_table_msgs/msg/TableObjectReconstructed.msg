# constants:
uint32 PLANE=1
uint32 SPHERE=2
uint32 CYLINDER=4
uint32 ROTATIONAL=8
uint32 BOX=16
uint32 MESH=32 # "else"

uint32 type  # one of the above
float64[] coefficients  # depending on shape type
float64 score # quality of fit

geometry_msgs/Point32[] vertices
int32[] triangles # indices into vertices list 

