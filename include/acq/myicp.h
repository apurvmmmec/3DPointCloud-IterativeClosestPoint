//
//  myicp.h
//  IGLFramework
//
//  Created by Apurv Nigam on 10/02/2017.
//
//


#ifndef myicp_h
#define myicp_h
#include "ANN/ANN.h"
using namespace std;
using namespace Eigen;

typedef void (*updateView)(vector<MatrixXd>);
typedef void (*colorUnMatched)(vector<int>);

vector<MatrixXd> icpPointToPoint(vector<MatrixXd> vecV, updateView uv,colorUnMatched paintUnMatched);
map<int,int>findClosestPoints(MatrixXd p, MatrixXd q, int maxPts);




#endif /* myicp_h */
