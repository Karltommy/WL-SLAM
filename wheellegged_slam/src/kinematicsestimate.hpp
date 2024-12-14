#pragma once
#ifndef KINEMATICSESTIMATE_H
#define KINEMATICSESTIMATE_H
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuFactor.h>




class Wheelleg
{
    private:

    double wheelradius_;
    double phi1_ , phi2_;
    double l1_, l2_; //l1大腿；l2小腿
    double body_length_;
    double body_width_;
    double distance_front_;
    double distance_height_;
    double h_body_;
    double H_;
    bool INIT=0;

    double pitch_value_;//front height minus rear height/body_length
    double roll_value_;//right height minus left height/body_width

    public:

   
    bool angle_updated=false;
    struct JointAngle
    {
        double T;
        double phi1_lf;
        double phi2_lf;
        double phi1_rf;
        double phi2_rf;
        double phi1_rr;
        double phi2_rr;
        double phi1_lr;
        double phi2_lr;
    };
    JointAngle JA_;


    // Wheelleg(double &phi1 , double &phi2, double &wheelradius, double &l1, double &l2)
    // :phi1_(phi1),phi2_(phi2),wheelradius_(wheelradius),l1_(l1),l2_(l2)
    // {}
    Wheelleg(){};
    
    Wheelleg(const double &l1, const double &l2, const double &wheelradius, const double &body_length, const double &body_width, const double &distance_front, const double &distance_height)
    :l1_(l1),l2_(l2),wheelradius_(wheelradius),body_length_(body_length),body_width_(body_width),distance_front_(distance_front),distance_height_(distance_height)
    {}

    void InitH();

    bool getINIT();

    void modifyINIT();

    double Onelegheight(double &phi1 , double &phi2);

    double Bodyheight(double &h_lf, double &h_rf, double &h_rr, double &h_lr);

    void resetangle(JointAngle &JA);

    void updateBodyheight();

    Wheelleg operator=(const Wheelleg &w1)
    {
        return w1;
    }

    double getBodyheight()
    {return h_body_;}

    double getdeltaheight()
    {return h_body_-H_;}

    double getinitheight()
    {return H_;}

    double getpitchvalue()
    {return pitch_value_;}

    double getrollvalue()
    {return roll_value_;}

    virtual ~Wheelleg();

   
    
};


class WlheightFactor: public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
    private:

    double height_;

    public:
    WlheightFactor(gtsam::Key poseKey, const double &height, gtsam::SharedNoiseModel model) :
    gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), height_(height){}

    gtsam::Vector evaluateError(const gtsam::Pose3& p, boost::optional<gtsam::Matrix&> H = boost::none) const
    {
        if (H) *H = (gtsam::Matrix16()<< 0.0,0.0,0.0,0.0, 0.0,1.0).finished();
        return (gtsam::Vector1()<<p.z() - height_).finished();
    }


};



#endif