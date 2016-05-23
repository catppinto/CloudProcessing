#ifndef cat_move_to_target_POS_ORI_REPRESENTATION_H
#define cat_move_to_target_POS_ORI_REPRESENTATION_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <Eigen/Dense>


class PosOriRepresentation
{
    public:
        PosOriRepresentation();
        virtual ~PosOriRepresentation();

        Eigen::MatrixXd RotationFromZYX(std::vector<double> zyx_eulerAngles);

        Eigen::MatrixXd RotationFromAngleAxis(std::vector<double> angle_axis);

        Eigen::MatrixXd RotationFromUnitQuaternion(std::vector<double> q);
        std::vector<double> ZYXfromRotationMatrix(Eigen::MatrixXd r);

        std::vector<double> AngleAxisfromRotationMatrix(Eigen::MatrixXd r);

        std::vector<double> UnitQuaternionfromRotationMatrix(Eigen::MatrixXd r);

        std::vector<double> UnitQuaternionfromAngleAxis(std::vector<double> angle_axis);

        std::vector<double> AngleAxisfromUnitQuaternion(std::vector<double> q);

    protected:
    private:

};


#endif
