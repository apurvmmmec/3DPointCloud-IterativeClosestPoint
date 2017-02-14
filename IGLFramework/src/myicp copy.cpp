//
//  myicp.cpp
//  IGLFramework
//
//  Created by Apurv Nigam on 10/02/2017.
//
//

#include <stdio.h>
#include <iostream>
//#include <igl/viewer/Viewer.h>
#include <igl/read_triangle_mesh.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/combine.h>
//#include <igl/viewer/Viewer.h>
#include <Eigen/Geometry>
#include <thread>
#include <acq/myicp.h>
#include <cstdlib>						// C standard library
#include <cstdio>						// C I/O (for sscanf)
#include <cstring>						// string manipulation
#include <fstream>						// file I/O
#include <ANN/ANN.h>					// ANN declarations

using namespace std;
using namespace Eigen;



vector<MatrixXd> myFunc(vector<MatrixXd> vecV){
    
    
    
    
    //vecV[1] =vecV[1];//* AngleAxisd(1/180.*3.14, Vector3d(0,1,0)).toRotationMatrix();
    MatrixXd p,q;
    int maxPts= 0;
    int nPoints0 =vecV[0].rows();
    int nPoints1 =vecV[1].rows();
    //    if(nPoints0>nPoints1){
    //        maxPts = nPoints1;
    //        p = vecV[0].topRows(nPoints1).transpose();
    //        q = vecV[1].transpose();
    //    }else{
    //        maxPts=nPoints0;
    //        p = vecV[0].transpose();
    //        q = vecV[1].topRows(nPoints0).transpose();
    //    }
    
    if(nPoints0>nPoints1){
        maxPts = 1000;
        p = vecV[0].topRows(1000).transpose();
        q = vecV[1].topRows(1000).transpose();
    }else{
        maxPts=1000;
        p = vecV[0].topRows(1000).transpose();
        q = vecV[1].topRows(1000).transpose();
    }
    
    MatrixXd avgP = p.rowwise().mean();
    MatrixXd avgQ = q.rowwise().mean();
    
    MatrixXd pNorm = p-avgP.replicate(1,p.cols());
    MatrixXd qNorm = q-avgQ.replicate(1,q.cols());
    
    MatrixXd C = pNorm*qNorm.transpose();
    JacobiSVD<MatrixXd> svd(C,ComputeThinU | ComputeThinV);
    //    cout<<svd.matrixU()<<endl;
    //    cout<<svd.matrixV()<<endl;
    MatrixXd R= svd.matrixV().transpose()*svd.matrixU().transpose();
    MatrixXd t= avgQ -R*avgP;
    
    ANNpointArray		dataPts;				// data points
    dataPts = annAllocPts(maxPts, 3);			// allocate data points
    ANNpoint			queryPt;				// query point
    queryPt = annAllocPt(3);					// allocate query point
    
    
    for (int r = 0 ; r< maxPts;r++){
        dataPts[r][0] = q.col(r)(0);
        dataPts[r][1] = q.col(r)(1);
        dataPts[r][2] = q.col(r)(2);
    }
    
//    cout<<"DataPts"<<endl;
//    for(int i=0;i<maxPts;i++){
//        cout<<dataPts[i][0]<<" ";
//    }
//    cout<<endl;
    int sz=maxPts;

    for (int i=0;i<maxPts;i++){
        queryPt[0] = p.col(i)(0);queryPt[1] = p.col(i)(1);queryPt[2] = p.col(i)(2);
        ANNpointArray dataPts1 = findClosestPoints(dataPts, queryPt,sz);
        sz=sz-1;
//        cout<<"DataPts1"<<endl;
//        for(int i=0;i<sz;i++){
//            cout<<dataPts1[i][0]<<" ";
//        }
//        cout<<endl;
        dataPts=dataPts1;
        
    }
        cout<<endl;
    //    cout<<nearNb[1]<<endl;
    
    
    return vecV;
    
    
}

ANNpointArray findClosestPoints(ANNpointArray vp, ANNpoint vq, int vmaxPts){
    
    ANNpointArray		dataPts;				// data points
    ANNpointArray       dataPts1;
    ANNpoint			queryPt;				// query point
    ANNidxArray			nnIdx;					// near neighbor indices
    ANNdistArray		dists;					// near neighbor distances
    ANNkd_tree*			kdTree;					// search structure
    int             maxPts          = vmaxPts;
    int				k				= 1;			// number of nearest neighbors
    int				dim				= 3;			// dimension
    double			eps				= 0;			// error bound
    nnIdx = new ANNidx[k];						// allocate near neigh indices
    dists = new ANNdist[k];						// allocate near neighbor dists
    dataPts=vp;
    queryPt=vq;
    
//    cout<<"in func dataPts"<<endl;
//    for(int i=0;i<maxPts;i++){
//        cout<<dataPts[i][0]<<" ";
//    }
//    cout<<endl;
//    cout<<"in func queryPt"<<endl;
//    cout<<queryPt[0]<<" ";
//    cout<<endl;
    
    kdTree = new ANNkd_tree(dataPts,maxPts,dim);
    kdTree->annkSearch(queryPt,k,nnIdx,dists,eps);
    
    cout << "\tNN:\tIndex\tDistance\n";
    cout << "\t" << 0;
    for (int j = 0; j < k; j++) {			// print summary
        dists[j] = sqrt(dists[j]);			// unsquare distance
        cout << "\t" << nnIdx[j] <<"\t"<<dists[j];
    }
    cout<<endl;
    
    dataPts1 = annAllocPts(maxPts-1,3);
    int c=0;
    for (int r = 0 ; r< maxPts;r++){
        if(r!= nnIdx[0]){
            dataPts1[c][0] = dataPts[r][0];
            dataPts1[c][1] = dataPts[r][1];
            dataPts1[c][2] = dataPts[r][2];
            c++;
        }
    }
    
    delete [] nnIdx;							// clean things up
    delete [] dists;
    delete kdTree;
    annClose();									// done with ANN
    
    return dataPts1;
}

//    Eigen::MatrixXd a(2,3);
//    a<<1,2,3,4,5,6;
//    cout<<a<<endl;
//    avgP= a.rowwise().mean();
//    cout<<avgP<<endl;

//        Eigen::MatrixXd b(3,1);
//        b<<3,4,5;
//        cout<<b<<endl;
//        Eigen::MatrixXd c =a-b.replicate(1,a.cols());
//        cout<<c<<endl;
//        cout<<c.transpose()<<endl;
//avgP(0,0) = p.row(0).mean();
//avgP(1,0) = p.row(1).mean();
//avgP(2,0) = p.row(2).mean();
