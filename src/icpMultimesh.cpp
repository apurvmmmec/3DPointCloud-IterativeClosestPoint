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
#include <ANN/ANN.h>

#include <acq/nearestNeighbour.h>
using namespace std;
using namespace Eigen;



// We align multiple meshes uding this function
// Since for ICP using nearest neighbour logic, the meshes that hav to aligned must be almost aligned, so I have applied rigid transformations to same mesh to create 5 mis aligned meshes.
// Let M1 , M2, M3 , M4 and M5 be the meshes.
//The logic I have used is to Align one by one each of M1, M2, M3 and M4 to M5 keeping M5 as fixed.
//Rest the complete logic is same as normal point to point ICP.
vector<MatrixXd> icpMultimesh(vector<MatrixXd> vecV,updateView uv){
  
    MatrixXd tempTrans(1,3);
    int nMesh = vecV.size();
    vecV[0] =vecV[0]*AngleAxisd(30/180.*3.14, Vector3d(0,1,0)).toRotationMatrix();
    vecV[1] =vecV[1]*AngleAxisd(60/180.*3.14, Vector3d(1,0,0)).toRotationMatrix();
    vecV[2] =vecV[2]*AngleAxisd(20/180.*3.14, Vector3d(0,0,1)).toRotationMatrix();
    vecV[3] =vecV[3]*AngleAxisd(300/180.*3.14, Vector3d(1,0,0)).toRotationMatrix();
    
    tempTrans.row(0)<< 0.101,0.103,0.105;
    vecV[0] = vecV[0]+tempTrans.replicate(vecV[0].rows(),1);
    
    tempTrans.row(0)<< 0.005,0.008,0.023;
    vecV[1] = vecV[1]+tempTrans.replicate(vecV[1].rows(),1);

    
    MatrixXd p,qq, avgP, avgQ, pNorm, qNorm,C,R,t;
    int nPoints;
    int numP =vecV[0].rows();
    int numQ =vecV[1].rows();
    int numR = vecV[2].rows();    map<int,int> nn;
    if(numP>numQ){
        nPoints = numQ;
    }else{
        nPoints = numP;
    }
    MatrixXd q(3,numP);
    
    
    int iter=0;
    double thresh= 1e-6;
    for (int k=0;k<nMesh-1;k++){
        double errNew=0;
        double err=1;
        while(abs(err-errNew)>thresh){
            iter=iter+1;
            cout<<"Iteration "<<iter;
            cout<<" Error delta="<<(err-errNew)<<endl;
            
            err= errNew;
            
            p  = vecV[k].transpose();
            
            nn = findClosestPoints(vecV[k],vecV[4]);
            for (int i=0;i<numP;i++)
            {
                q.col(i)=(vecV[4].transpose()).col(nn.at(i));
            }
            
            avgP = p.rowwise().mean();
            avgQ = q.rowwise().mean();
            
            pNorm = p-avgP.replicate(1,p.cols());
            qNorm = q-avgQ.replicate(1,q.cols());
            
            C = pNorm*qNorm.transpose();
            JacobiSVD<MatrixXd> svd(C,ComputeThinU | ComputeThinV);
            R= svd.matrixV()*svd.matrixU().transpose();
            t= avgQ -R*avgP;
            
            vecV[k]=(R*(vecV[k].transpose())+t.replicate(1,vecV[k].transpose().cols())).transpose();
            errNew = (vecV[k].topRows(nPoints)-q.transpose().topRows(nPoints)).squaredNorm();
                       cout<<"Error = "<<errNew<<endl;
            
            
            (*uv)(vecV);
        }
        
        cout<<"Converged "<<", delta is ="<<abs(err-errNew)<<endl;
    }
    
    return vecV;
}

