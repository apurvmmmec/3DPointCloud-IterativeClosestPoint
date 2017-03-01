//
//  myicp.cpp
//  IGLFramework
//
//  Created by Apurv Nigam on 10/02/2017.
//
//

#include <stdio.h>
#include <iostream>
#include <igl/read_triangle_mesh.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/combine.h>
#include <Eigen/Geometry>
#include <thread>
#include <acq/myicp.h>
#include <cstdlib>						// C standard library
#include <cstdio>						// C I/O (for sscanf)
#include <cstring>						// string manipulation
#include <fstream>						// file I/O
#include <ANN/ANN.h>					// ANN declarations
#include<random>
#include<chrono>
#include <acq/nearestNeighbour.h>
#include <acq/icpSubsampled.h>
using namespace std;
using namespace Eigen;


// We do ICP over here not using all the points in P and Q but a sub sample of points from P and Q

// We get additional paramter ssFactor which tells how much to subsample. Its  greater than 0 and less then equal to 1
vector<MatrixXd> icpSubsampled(vector<MatrixXd> vecV,float ssFactor, updateView uv){
  
    // Variable to determine howw many points we have to skip.
    //Like if ssFactor is 0.5, we skip 1/0.5 i.e every 2nd point from P and Q while collecting samples.
    //This helps us uniformly collect samples
    int ssFact = (1/ssFactor);
    cout<<"Subsampling by"<<ssFactor<<endl;
    
    MatrixXd p,qq, avgP, avgQ, pNorm, qNorm,C,R,t;
    int nPoints;
    int numP =vecV[0].rows();
    int numQ =vecV[1].rows();
    double errNew=0;
    double err=1;
    map<int,int> nn;
    
    // We form our subsampled matrices that we use while doing the ICP.
    //Rest all of the process is same as normal ICP
    int szP = numP/ssFact;
    MatrixXd ssP(szP,3);
    for (int j=0;j<szP;j++){
        ssP.row(j) = vecV[0].row(j*ssFact);
    }
    
    int szQ = numQ/ssFact;
    MatrixXd ssQ(szQ,3);
    for (int  j=0;j<szQ;j++){
        ssQ.row(j) = vecV[1].row(j*ssFact);
    }
    
    
    if(szP>szQ){
        nPoints = szQ;
    }else{
        nPoints = szP;
    }
    
    MatrixXd q(3,szP);

    int iter=0;
    double thresh= 1e-6;
    while(abs(err-errNew)>thresh){
        iter=iter+1;
        cout<<"Iteration "<<iter;
        cout<<" Error delta="<<(err-errNew)<<endl;

        err= errNew;

        
        p  = ssP.transpose();
        
        nn = findClosestPoints(ssP,ssQ);
        for (int i=0;i<szP;i++)
        {
            q.col(i)=(ssQ.transpose()).col(nn.at(i));
        }
        
        avgP = p.rowwise().mean();
        avgQ = q.rowwise().mean();
        
        pNorm = p-avgP.replicate(1,p.cols());
        qNorm = q-avgQ.replicate(1,q.cols());
        
        C = pNorm*qNorm.transpose();
        JacobiSVD<MatrixXd> svd(C,ComputeThinU | ComputeThinV);
        R= svd.matrixV()*svd.matrixU().transpose();
        t= avgQ -R*avgP;
        
        vecV[0]=(R*(vecV[0].transpose())+t.replicate(1,vecV[0].transpose().cols())).transpose();
        for (int j=0;j<szP;j++){
            ssP.row(j) = vecV[0].row(j*ssFact);
        }

        errNew = (ssP.topRows(nPoints)-q.transpose().topRows(nPoints)).squaredNorm();

        cout<<"Error = "<<errNew<<endl;
        
        
        (*uv)(vecV);
    }
    cout<<"Converged "<<", delta is ="<<abs(err-errNew)<<endl;
    
    return vecV;
}
