#include "states.h"

// constructor
States::States()
{
    X = Vec3d(0,0,0);
    V = Vec3d(0,0,0);
    nf = 5;
    features.reserve(nf);
    b = Vec3d(0,0,0);

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
}

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
}

void States::setb(Vec3d bias)
{
    b = bias;
}
