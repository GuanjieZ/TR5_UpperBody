/////////////////////////////////////////////////////////
// NOTE: Z-axis pointing down, X-axis pointing forward //
/////////////////////////////////////////////////////////

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

Eigen::Vector2f solveIK_LH(Eigen::Vector2f);
Eigen::Vector2f solveFK_LH(Eigen::Vector2f theta_init, Eigen::Vector2f q_meas);

Eigen::Vector2f solveIK_RH(Eigen::Vector2f);
Eigen::Vector2f solveFK_RH(Eigen::Vector2f theta_init, Eigen::Vector2f q_meas);

enum class Side : unsigned char { Left, Right};
std::pair<float, float> solveFK(float roll_init, float pitch_init, float f_len, float b_len, Side side);
std::pair<float, float> solveIK(float roll, float pitch, Side side);