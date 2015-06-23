#include "states.h"
#define DMIN 0.3            /*  */
using std::cout;
using std::endl;

    States&
States::operator*= ( const double& rhs )
{
    this->X*=rhs;
    this->V*=rhs;
    for( Fiter fi=this->features.begin(); fi!=this->features.end(); ++fi ) 
        fi->set_body_position( fi->get_body_position()*rhs );
    return *this;
}
// constructor

    States&
States::operator+= ( const States& rhs )
{
    this->add(rhs);
    return *this;
}

States::States ( const cv::Mat& kx ) :
    X(cv::Vec3d( kx.at<double>(0,0), kx.at<double>(1,0), kx.at<double>(2,0)) ),
    V( cv::Vec3d( kx.at<double>(3,0), kx.at<double>(4,0), kx.at<double>(5,0)) )
    //b( cv::Vec3d( kx.at<double>(6,0), kx.at<double>(7,0), kx.at<double>(8,0) ) )
{
    int nf;
    nf = (kx.rows-9)/3;
    for( int i=0; i<nf; ++i )
    {
        addFeature(Feature( cv::Vec3d( kx.at<double>(6+3*i,0), 
            kx.at<double>(7+3*i,0), kx.at<double>(8+3*i,0) ), 0 )); 
    }
    return ;
}		/* -----  end of method States::States  ----- */

// accessor
Vec3d States::getX() { return X; }

Vec3d States::getV() { return V; }

Vec3d States::getb() { return b; }

// mutator
States& States::setX(const Vec3d& pos) { 
    X = pos; 
    return *this;
}

States& States::setV(const Vec3d& vel) { 
    V = vel; 
    return *this;
}

States& States::addFeature(const Feature& f)
{
    features.push_back(f);
    return *this;
}

States& States::setb(const Vec3d& bias)
{
    b = bias;
    return *this;
}

void States::add( const States& a)
{
    if( a.features.size()!=this->features.size() )
    {
        std::cerr << "add: feature mismatch" << std::endl;
        exit(EXIT_FAILURE);
    }
    this->X += a.X;
    this->V += a.V;
    for (size_t i = 0; i<features.size(); i++ )
    {
        this->features[i].set_body_position( this->features[i].get_body_position() +
                a.features[i].get_body_position() );
        //this->features[i].set_world_position( this->features[i].get_world_position() +
         //       a.features[i].get_world_position() );
    }
    this->b += a.b;
}

    void
States::update_features ( const ImageSensor& imgsense, const Sensors& sense, cv::Mat& P )
{
    const double d_min=0.1;
    const double d_max=10e3;
    // Go through current features and mark the ones that are inactive.
    for (Fiter fi=features.begin();
            fi!=features.end(); ++fi) {
        bool found=false;
        cv::Vec3d pos=fi->get_body_position();
        if (pos[2]<1/d_max) {
            pos[2]=1/d_max;
        } else if (pos[2]>1/d_min) {
            pos[2]=1/d_min;
        }
        fi->set_body_position(pos);
        for (cMatchIter match=imgsense.matches.begin();
                match!=imgsense.matches.end(); ++match) {
            if (fi->getID()==match->id) {
                fi->set_noMatch(0);
                found=true;
                break;
            }
        }
        if (found==false) {
            fi->set_noMatch(1);
        }
    }
    // Go through the new features and replace inactive features with new
    // features. Expand the vector if necessary.
    for (cMatchIter match=imgsense.matches.begin();
            match!=imgsense.matches.end(); ++match) {
        bool found=false;
        for (Fiter fi=features.begin();
                fi!=features.end(); ++fi) {
            if (fi->getID()==match->id) {
                found=true;
                break;
            }
        }
        if (found==true) continue;
        int i=0;
        Fiter fi=features.begin();
        for (; fi!=features.end(); ++fi, ++i) {
            if (fi->get_noMatch()!=0) {
                *fi = Feature( X, sense, *match);
                P.rowRange(6+3*i,6+3*i+3).setTo(0);
                P.colRange(6+3*i,6+3*i+3).setTo(0);
                P(Rect(6+3*i,6+3*i,3,3))=2*Mat::eye(3,3,CV_64F);
                found=true;
                break;
            }
        }
        if (found==false) {
            features.emplace_back(X, sense, *match);
            // Expand the P vector.
            int nf=features.size();
            cv::Mat bigP=cv::Mat::zeros(9+3*nf,9+3*nf,CV_64F);
            blockAssign(bigP, P, cv::Point(0,0));

            // Move the bias vals out to the edge
            cv::Mat biasrow,biascol,newbiasrow,newbiascol;
            biasrow=bigP(cv::Rect(0,6+3*(nf-1),6+3*(nf-1),3));
            biascol=bigP(cv::Rect(6+3*(nf-1),0,3,6+3*(nf-1)));
            newbiasrow=bigP(cv::Rect(0,6+3* nf,6+3*(nf-1),3));
            newbiascol=bigP(cv::Rect(6+3*nf, 0,3,6+3*(nf-1)));
            biasrow.copyTo(newbiasrow);
            biascol.copyTo(newbiascol);
            // Move the bias diagonal to the new edge diagonal;
            cv::Mat biasdiag, newbiasdiag;
            biasdiag=bigP(cv::Rect(6+3*(nf-1),6+3*(nf-1),3,3));
            newbiasdiag=bigP(cv::Rect(6+3*nf,6+3*nf,3,3));
            biasdiag.copyTo(newbiasdiag);
            // Initialize the new features
            biasrow.setTo(0);
            biascol.setTo(0);
            blockAssign(biasdiag,2*cv::Mat::eye(3,3,CV_64F),cv::Point(0,0));
            P=bigP;
        }
    }
    // Go through the features vector, look for any features that were lost and
    // not replaced. Remove them from the features vector and contract P.
    Fiter fi=features.begin();
    int i=0;
    while (fi!=features.end()) {
        int nf=features.size();
        if (fi->get_noMatch()==0) {
            ++fi;
            ++i;
            continue;
        }
        // Zero out the row and column of the feature
        P.rowRange(6+3*i,6+3*i+3).setTo(0);
        P.colRange(6+3*i,6+3*i+3).setTo(0);

        cv::Mat lowleft, upright, lowright;
        lowleft=P(cv::Rect(0,6+3*i+3,6+3*i,(9+3*nf)-(6+3*i+3)));
        upright=P(cv::Rect(6+3*i+3,0,(9+3*nf)-(6+3*i+3),6+3*i));
        lowright=P(cv::Rect(6+3*i+3,6+3*i+3,(9+3*nf)-(6+3*i+3),(9+3*nf)-(6+3*i+3)));

        // Shift values in lower left rows up
        blockAssign(P,lowleft,cv::Point(0,6+3*i));
        // Shift values in upper right to the left
        blockAssign(P,upright,cv::Point(6+3*i,0));
        // Shift values in lower right up and left
        blockAssign(P,lowright,cv::Point(6+3*i,6+3*i));

        // Contract ROI
        P=P(cv::Rect(0,0,9+3*(nf-1),9+3*(nf-1)));

        // Remove feature from features vector.
        fi=features.erase(fi);
        ++i;
    }
            
    // Determine rf, nrf
    rf=nrf=0;
    for (Fiter fi=features.begin();
            fi!=features.end(); ++fi) {
        if (fi->initial.isRef) {
            rf++;
        } else {
            nrf++;
        }
    }
    return ;
}		/* -----  end of method States::update_features  ----- */

/*
 *--------------------------------------------------------------------------------------
 *       Class:  States
 *      Method:  States :: dynamics
 * Description:  Apply motion model to state and return predicted state.
 *--------------------------------------------------------------------------------------
 */
    States
States::dynamics ( const Sensors& s, double dt )
{
    States predicted_state;
    Matx33d A;
    cv::Vec3d w;
    Matx33d Rb2w, Rw2b; 

    Rb2w = s.quat.get_value().rotation();
    Rw2b = Rb2w.t();

    //w =200*s.ang.get_value();
    w =s.ang.get_value();

    cv::Vec3d gw(0,0,-GRAVITY); 
    A = cv::Matx33d( 0, -w[2], w[1],
            w[2], 0, -w[0],
            -w[1], w[0], 0 );
    
    // Generalized matrix multiplication
    gemm( Rb2w, V, 1, Mat(), 0, predicted_state.X );
    //gemm( -A, V, 1, 200*s.acc.get_value(), 1, predicted_state.V );
    gemm( -A, V, 1, s.acc.get_value(), 1, predicted_state.V );
    gemm( Rw2b, gw, -1, predicted_state.V, 1, predicted_state.V);
    //predicted_state.V=200*s.acc.get_value()-b;
    //predicted_state.V=s.acc.get_value()-b;
    //gemm( Rw2b, gw, 1, predicted_state.V, 1, predicted_state.V);

    Fiter pib=features.begin();
    for( int i=0; pib!=features.end(); ++pib,++i )
    {
        Feature fi;
        cv::Vec3d bp;
        bp = pib->get_body_position();
        fi.set_body_position( cv::Vec3d( 
            (-V[1] + bp[0]*V[0])*bp[2] + bp[1]*w[0] - (1 + bp[0]*bp[0])*w[2] + bp[0]*bp[1]*w[1],
            (-V[2] + bp[1]*V[0])*bp[2] - bp[0]*w[0] + (1 + bp[1]*bp[1])*w[1] - bp[0]*bp[1]*w[2],
            (-w[2] * bp[0]+w[1] *bp[1])* bp[2]+V[0] *      bp[2]*bp[2])
        );
        predicted_state.addFeature(fi);
    }
    predicted_state.V-=b;
    predicted_state.b=cv::Vec3d(0,0,0);
    return predicted_state*dt;
}		/* -----  end of method States::dynamics  ----- */


    int
States::getRows ( ) const
{
    return 9+3*getNumFeatures() ;
}		/* -----  end of method States::getRows  ----- */


    int
States::getNumFeatures ( ) const
{
    return features.size();
}		/* -----  end of method States::getNumFeatures  ----- */

