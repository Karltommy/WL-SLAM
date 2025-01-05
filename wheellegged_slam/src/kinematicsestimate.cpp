#include "kinematicsestimate.hpp"
void Wheelleg::InitH()
{
    H_=h_body_;
}

bool Wheelleg::getINIT()
{
    return INIT;
}

void Wheelleg::modifyINIT()
{
    INIT=!INIT;
}

double Wheelleg::Onelegheight(double &phi1 , double &phi2)
{
        phi1_=phi1;
        // phi2_=phi1+phi2-180;
        phi2_=180-phi2;
        double h = l1_*std::sin(phi1_*M_PI/180)+l2_*std::sin(phi2_*M_PI/180)+wheelradius_;
        return h;
}

double Wheelleg::Bodyheight(double &h_lf, double &h_rf, double &h_rr, double &h_lr)
{
    double h_left;
    double h_right;
    double h_body;
    h_left = h_lf+(h_lf-h_lr)*(1-distance_front_/body_length_)+(sqrt(body_length_*body_length_-(h_lf-h_lr)*(h_lf-h_lr))/body_length_)*distance_height_;
    h_right = h_rf+(h_rf-h_rr)*(1-distance_front_/body_length_)+(sqrt(body_length_*body_length_-(h_rf-h_rr)*(h_rf-h_rr))/body_length_)*distance_height_;
    h_body = (h_left+h_right)/2;
    return h_body;
}

void Wheelleg::resetangle(JointAngle &JA)
{
    JA_ = JA;
}

void Wheelleg::updateBodyheight()
{
    double h_lf = Onelegheight(JA_.phi1_lf,JA_.phi2_lf);
    double h_rf = Onelegheight(JA_.phi1_rf,JA_.phi2_rf);
    double h_rr = Onelegheight(JA_.phi1_rr,JA_.phi2_rr);
    double h_lr = Onelegheight(JA_.phi1_lr,JA_.phi2_lr);
    h_body_ = Bodyheight(h_lf, h_rf, h_rr, h_lr);
    pitch_value_ = (h_lf+h_rf-h_rr-h_lr)/2/body_length_;
    roll_value_ = (h_rr+h_rf-h_lf-h_lr)/2/body_width_;
}


Wheelleg::~Wheelleg()
{}

