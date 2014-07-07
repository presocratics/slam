#include "states.h"

    States&
States::operator*= ( const double& rhs )
{
    this->X*=rhs;
    this->V*=rhs;
    for( Fiter fi=this->features.begin(); fi!=this->features.end(); ++fi ) fi->position.body*=rhs;
    return *this;
}
// constructor

    States&
States::operator+= ( const States& rhs )
{
    this->add(rhs);
    return *this;
}
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

States::States ( cv::Mat kx )
{
    int nf;
    nf = (kx.rows-9)/3;
    setX( cv::Vec3d( kx.at<double>(0,0), kx.at<double>(1,0), kx.at<double>(2,0)) );
    setV( cv::Vec3d( kx.at<double>(3,0), kx.at<double>(4,0), kx.at<double>(5,0)) );
    for( int i=0; i<nf; ++i )
    {
        Feature ft( cv::Vec3d( kx.at<double>(9+3*i,0), kx.at<double>(10+3*i,0), kx.at<double>(11+3*i,0) ),
                cv::Scalar(0,0,0), 0 );
        addFeature(ft);
    }
    setb( cv::Vec3d( kx.at<double>(6,0), kx.at<double>(7,0), kx.at<double>(8,0) ) );
    return ;
}		/* -----  end of method States::States  ----- */

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
            features[i].position.body += a.features[i].position.body;
            features[i].position.world += a.features[i].position.world;
        }
        b += a.b;
    }
}

    void
States::update_features ( ImageSensor *imgsense, Sensors sense )
{
    features.clear();
    // Age each feature
    featIter fi=feats.begin(); 
    for( ; fi!=feats.end(); ++fi )
    {
        fi->second.incNoMatch();
    }

    matchIter match=imgsense->matches.begin();
    for( ; match!=imgsense->matches.end(); ++match )
    {
        Feature f;
        fi=feats.find(match->id);
        if( fi==feats.end() ) // New feature
        {
            f=Feature( X, sense, match);
            feats.insert( std::pair<int,Feature>(match->id, f) );
        }
        else if( fi->second.get_noMatch()>1 ) // Reinitialize old feature
        {
            fi->second.initialize( X, sense, match, true );
            f=fi->second;
        }
        else
        {
            fi->second.set_noMatch(0);
            f=fi->second;
        }
        features.push_back(f);
    }
    return ;
}		/* -----  end of method States::update_features  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  States
 *      Method:  States :: end_loop
 * Description:  Terrible hack copying vector into map.
 *--------------------------------------------------------------------------------------
 */
    void
States::end_loop (  )
{
    for( Fiter vf=features.begin(); vf!=features.end(); ++vf )
    {
        featIter fi;
        if( (fi=feats.find(vf->ID))==feats.end() )
        {
            std::cerr << "end_loop: feat not found." <<std::endl;
            exit(EXIT_FAILURE);
        }
        fi->second=*vf;
    }

    return ;
}		/* -----  end of method States::end_loop  ----- */


/*
 *--------------------------------------------------------------------------------------
 *       Class:  States
 *      Method:  States :: dynamics
 * Description:  Apply motion model to state and return predicted state.
 *--------------------------------------------------------------------------------------
 */
    States
States::dynamics ( Sensors s, bool flagbias )
{
    States predicted_state;
    Matx33d A;
    cv::Vec3d w;
    Matx33d Rb2w, Rw2b; 

    Rb2w = s.quaternion.rotation();
    Rw2b = Rb2w.t();

    w =cv::Vec3d(s.angular_velocity);

    Vec3d gw(0,0,GRAVITY); // TODO where does this number come from?
    A = Matx33d( 0, -w[2], w[1],
            w[2], 0, -w[0],
            -w[1], w[0], 0 );
    
    // Generalized matrix multiplication
    gemm( Rb2w, V, 1, Mat(), 0, predicted_state.X );

    gemm( -A, V, 1, s.acceleration, 1, predicted_state.V );
    gemm( Rw2b, gw, -1, predicted_state.V, 1, predicted_state.V);

    std::vector<Feature>::iterator pib=features.begin();
    for( int i=0; pib!=features.end(); ++pib,++i )
    {
        Feature fi;
        cv::Vec3d bp;
        bp = pib->get_body_position();
        fi.position.body = cv::Vec3d( 
            (-V[1] + bp[0]*V[0])*bp[2] + bp[1]*w[0] - (1 + bp[0]*bp[0])*w[2] + bp[0]*bp[1]*w[1],
            (-V[2] + bp[1]*V[0])*bp[2] - bp[0]*w[0] + (1 + bp[1]*bp[1])*w[1] - bp[0]*bp[1]*w[2],
            (-w[2] * bp[0]+w[1] *bp[1])* bp[2]+V[0] *      bp[2]*bp[2]
        );
        predicted_state.addFeature(fi);
    }
    if( flagbias==true ) 
    {
        V-=b;
        b=cv::Vec3d(0,0,0);
    }
    return predicted_state;
}		/* -----  end of method States::dynamics  ----- */


    int
States::getRows ( )
{
    return 6+3*getNumFeatures()+3 ;
}		/* -----  end of method States::getRows  ----- */


    int
States::getNumFeatures ( )
{
    return features.size();
}		/* -----  end of method States::getNumFeatures  ----- */

