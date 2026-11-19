# All lengths are metres; header is the image acquisition time.
std_msgs/Header header
string target
string method
string depth_source
string reference_point
geometry_msgs/Point position
# Ignore orientation unless orientation_valid is true.
bool orientation_valid
geometry_msgs/Quaternion orientation
# Order x,y,z,rot_x,rot_y,rot_z. Unknown rotation variances are +inf.
float64[36] covariance
# Cylinder axis is an unoriented line: a and -a describe the same axis.
bool axis_valid
geometry_msgs/Vector3 axis
bool radius_valid
float64 radius
string symmetry
float64 fit_rmse
uint32 support
