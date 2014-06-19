#include "states.h"

// constructor
States::States()
{
    X = Vec3d(0,0,0);
    V = Vec3d(0,0,0);
    nf = 5;
    features.reserve(nf);
    b = Vec3d(0,0,0);
    rows = 6+3*nf+3;
}

States::~States()
{
    features.clear();
}

States::States(Vec3d pos, Vec3d vel, std::vector<Feature> feat, Vec3d bias, int n)
{
    X = pos;
    V = vel;
    features = feat;
    b = bias;
    nf = n;
    rows = 6+3*nf+3;

}

// accessor
Vec3d States::getX()
{
    return X;
}

Vec3d States::getV()
{
    return V;
}

std::vector<Feature> States::getFeatures()
{
    return features;
}

Feature States::getFeature(int i)
{
    return features[i];
}

Vec3d States::getb()
{
    return b;
}

// mutator
void States::setX(Vec3d pos)
{
    X = pos;
}

void States::setV(Vec3d vel)
{
    V = vel;
}

void States::setFeature(int i, Feature f)
{
    if(features.size() > i)
    {
        std::swap(features[features.size()-1], features[i]);
        features.pop_back();
    }
}

void States::addFeature(Feature f)
{
    features.push_back(f);
    nf = features.size();
    rows = 6+nf*3+3;
}

void States::setb(Vec3d bias)
{
    b = bias;
}

void States::add(States a)
{
    if(rows == a.rows)
    {
        X += a.X;
        V += a.V;
        for(int i = 0; i < nf; i++ )
        {
            features[i].X += a.features[i].X;
        }
        b += a.b;
    }
}


/*
 *--------------------------------------------------------------------------------------
 *       Class:  States
 *      Method:  States :: dynamics
 * Description:  Apply motion model to state and return predicted state.
 *--------------------------------------------------------------------------------------
 */
    States
States::dynamics (Quaternion qbw, cv::Vec3d a, cv::Vec3d w )
{
    States predicted_state;
    Matx33d A;
    
    Matx33d Rb2w, Rw2b; 
    Rb2w = qbw.rotation();
    Rw2b = Rb2w.t();

    Vec3d gw(0,0,-9.80665); // TODO where does this number come from?
    A = Matx33d( 0, -w[2], w[1],
            w[2], 0, -w[0],
            -w[1], w[0], 0 );
    
    // Generalized matrix multiplication
    gemm( Rb2w, V, 1, Mat(), 0, predicted_state.X );

    gemm( -A, V, 1, a, 1, predicted_state.V );
    gemm( Rw2b, gw, -1, predicted_state.V, 1, predicted_state.V);
    return predicted_state;
}		/* -----  end of method States::dynamics  ----- */
